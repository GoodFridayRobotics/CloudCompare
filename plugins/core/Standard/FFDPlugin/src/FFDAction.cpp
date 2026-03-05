//##########################################################################
//#                                                                        #
//#                   CLOUDCOMPARE PLUGIN: FFDPlugin                       #
//#           Free Form Deformation - Non-rigid Transformation             #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//##########################################################################

#include "FFDAction.h"
#include "FFDLattice.h"
#include "FFDLatticeParamsDlg.h"
#include "ccFFDLatticeDisplay.h"
#include "ccFFDDeformationTool.h"

#include <ccMainAppInterface.h>
#include <ccPointCloud.h>
#include <ccHObjectCaster.h>
#include <ccGLWindowInterface.h>
#include <ccHObject.h>
#include <ReferenceCloud.h>

#include <QObject>
#include <algorithm>
#include <vector>

#include "FFDDebug.h"

namespace FFDAction
{
	// Track the currently active FFD tool to prevent duplicates
	static ccFFDDeformationTool* s_activeTool = nullptr;

	//! Find an object by unique ID in the DB tree
	static ccHObject* findByUniqueID(ccHObject* root, unsigned int uniqueID)
	{
		if (!root)
			return nullptr;
		if (root->getUniqueID() == uniqueID)
			return root;
		for (unsigned i = 0; i < root->getChildrenNumber(); ++i)
		{
			ccHObject* found = findByUniqueID(root->getChild(i), uniqueID);
			if (found)
				return found;
		}
		return nullptr;
	}

	void performDeformation( ccMainAppInterface *appInterface )
	{
		if ( appInterface == nullptr )
		{
			Q_ASSERT( false );
			return;
		}

		// Stop any existing FFD tool before creating a new one
		if (s_activeTool)
		{
			FFD_DEBUG("performDeformation: Stopping existing tool=" << s_activeTool);
			s_activeTool->stop(false);
			s_activeTool->deleteLater();
			s_activeTool = nullptr;
			FFD_DEBUG("performDeformation: Old tool stopped and scheduled for deletion");
		}

		/*** HERE STARTS THE ACTION ***/

		// Get the selected cloud
		ccHObject::Container selectedEntities = appInterface->getSelectedEntities();

		if ( selectedEntities.empty() )
		{
			appInterface->dispToConsole( "[FFD] No entity selected", ccMainAppInterface::WRN_CONSOLE_MESSAGE );
			return;
		}

		// Find the first point cloud
		ccPointCloud *cloud = nullptr;
		for ( ccHObject *entity : selectedEntities )
		{
			if ( entity->isA( CC_TYPES::POINT_CLOUD ) )
			{
				cloud = static_cast<ccPointCloud*>( entity );
				break;
			}
		}

		if ( cloud == nullptr )
		{
			appInterface->dispToConsole( "[FFD] No point cloud selected", ccMainAppInterface::ERR_CONSOLE_MESSAGE );
			return;
		}

		// Show dialog to get lattice parameters
		FFDLatticeParamsDlg paramsDlg(nullptr);

		// Set up live lattice preview in the 3D view
		ccGLWindowInterface* win = appInterface->getActiveGLWindow();
		if (win)
		{
			paramsDlg.setPreviewContext(cloud->getOwnBB(), win);
		}

		// Collect available point clouds from DB tree for trajectory selection
		{
			ccHObject* dbRoot = appInterface->dbRootObject();
			if (dbRoot)
			{
				ccHObject::Container pointClouds;
				dbRoot->filterChildren(pointClouds, true, CC_TYPES::POINT_CLOUD, true);
				std::vector<ccPointCloud*> candidates;
				for (ccHObject* entity : pointClouds)
				{
					ccPointCloud* pc = ccHObjectCaster::ToPointCloud(entity);
					if (pc)
						candidates.push_back(pc);
				}
				paramsDlg.setAvailableTrajectories(candidates, cloud);
			}
		}

		if (paramsDlg.exec() != QDialog::Accepted)
		{
			appInterface->dispToConsole( "[FFD] Cancelled by user", ccMainAppInterface::WRN_CONSOLE_MESSAGE );
			return;
		}

		// Create the FFD lattice with user-specified dimensions
		std::array<unsigned int, 3> latticeSize = paramsDlg.getLatticeSize();
		DeformationType deformType = paramsDlg.getDeformationType();
		double rotationZ = paramsDlg.getRotationZ();
		ccPointCloud* trajectoryCloud = paramsDlg.getSelectedTrajectory();
		FFDLattice* lattice = new FFDLattice(latticeSize, cloud->getOwnBB(), rotationZ);
		lattice->setDeformationType(deformType);

		// Create a subsampled preview cloud for smooth interactive updates
		size_t fullSize = cloud->size();
		size_t targetPreviewSize = paramsDlg.getPreviewPointCount();
		size_t step = (fullSize > targetPreviewSize && targetPreviewSize > 0) ? (fullSize / targetPreviewSize) : 1;
		step = std::max<size_t>(step, 1);

		CCCoreLib::ReferenceCloud previewRef(cloud);
		previewRef.reserve(static_cast<unsigned>(fullSize / step + 1));
		for (size_t i = 0; i < fullSize; i += step)
		{
			previewRef.addPointIndex(static_cast<unsigned>(i));
		}

		ccPointCloud* previewCloud = cloud->partialClone(&previewRef);
		if (!previewCloud)
		{
			// Fallback: create a minimal preview cloud if cloning failed
			previewCloud = new ccPointCloud("FFD Preview");
			previewCloud->reserve(previewRef.size());
			for (unsigned i = 0; i < previewRef.size(); ++i)
			{
				const CCVector3* P = cloud->getPoint(previewRef.getPointGlobalIndex(i));
				previewCloud->addPoint(*P);
			}
		}
		previewCloud->setName(QString("%1 (FFD Preview)").arg(cloud->getName()));
		previewCloud->setVisible(true);
		previewCloud->setEnabled(true);
		previewCloud->setPointSize(cloud->getPointSize());

		// Visualize lattice
		ccFFDLatticeDisplay* latticeDisplay = new ccFFDLatticeDisplay(cloud->getOwnBB(), latticeSize, lattice->getAllControlPoints());
		latticeDisplay->setName("FFD Lattice");
		latticeDisplay->setLatticeConfig(rotationZ, static_cast<int>(deformType), cloud->getUniqueID());

		// Create a group entity to host the preview and lattice
		ccHObject* ffdGroup = new ccHObject("FFD");
		ffdGroup->addChild(previewCloud);
		ffdGroup->addChild(latticeDisplay);

		appInterface->addToDB(ffdGroup, false, true, false, true);
		appInterface->refreshAll();

		appInterface->dispToConsole(
			QString("[FFD] Preview cloud size: %1 / %2 points")
				.arg(previewCloud->size())
				.arg(fullSize),
			ccMainAppInterface::STD_CONSOLE_MESSAGE
		);

		appInterface->dispToConsole(
			QString( "[FFD] Created lattice with %1x%2x%3 control points" )
				.arg( latticeSize[0] )
				.arg( latticeSize[1] )
				.arg( latticeSize[2] ),
			ccMainAppInterface::STD_CONSOLE_MESSAGE
		);

		// Create and setup the interactive deformation tool
		FFD_DEBUG("performDeformation: Creating new ccFFDDeformationTool...");
		ccFFDDeformationTool* tool = new ccFFDDeformationTool(cloud, previewCloud, appInterface);
		FFD_DEBUG("performDeformation: tool created=" << tool);
		s_activeTool = tool;

		if (trajectoryCloud)
		{
			tool->setTrajectoryCloud(trajectoryCloud);
			appInterface->dispToConsole(
				QString("[FFD] Associated trajectory: %1 (%2 points)")
					.arg(trajectoryCloud->getName())
					.arg(trajectoryCloud->size()),
				ccMainAppInterface::STD_CONSOLE_MESSAGE);
		}

		tool->setLattice(lattice, latticeDisplay);
		FFD_DEBUG("performDeformation: lattice set");

		if (!win)
		{
			win = appInterface->getActiveGLWindow();
		}
		if (!win)
		{
			appInterface->dispToConsole( "[FFD] No active 3D window", ccMainAppInterface::ERR_CONSOLE_MESSAGE );
			delete tool;
			return;
		}

		tool->linkWith(win);
		FFD_DEBUG("performDeformation: tool linked with win=" << win);
		tool->start();
		FFD_DEBUG("performDeformation: tool started");

		QObject::connect(tool, &ccOverlayDialog::processFinished, [tool](bool)
		{
			FFD_DEBUG("processFinished: tool=" << tool << " being scheduled for deletion");
			if (s_activeTool == tool)
				s_activeTool = nullptr;
			tool->deleteLater();
		});

		appInterface->dispToConsole( "[FFD] Interactive tool activated. Press X/Y/Z to constrain movement to an axis.",
									  ccMainAppInterface::STD_CONSOLE_MESSAGE );
		appInterface->dispToConsole( "[FFD] Press Enter to apply deformation, R to reset.",
									  ccMainAppInterface::STD_CONSOLE_MESSAGE );

		/*** HERE ENDS THE ACTION ***/
	}

	void editLattice( ccMainAppInterface *appInterface )
	{
		if ( !appInterface )
			return;

		// Stop any existing FFD tool first
		if (s_activeTool)
		{
			s_activeTool->stop(false);
			s_activeTool->deleteLater();
			s_activeTool = nullptr;
		}

		// Find the selected FFD lattice display
		ccHObject::Container selectedEntities = appInterface->getSelectedEntities();
		ccFFDLatticeDisplay* latticeDisplay = nullptr;
		for ( ccHObject *entity : selectedEntities )
		{
			if ( ccFFDLatticeDisplay::IsFFDLatticeDisplay(entity) )
			{
				latticeDisplay = static_cast<ccFFDLatticeDisplay*>(entity);
				break;
			}
		}
		if ( !latticeDisplay )
		{
			appInterface->dispToConsole("[FFD] No FFD lattice display selected", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}

		// Look up the original cloud by unique ID
		unsigned int cloudUID = latticeDisplay->getOriginalCloudUniqueID();
		ccHObject* dbRoot = appInterface->dbRootObject();
		ccHObject* foundObj = findByUniqueID(dbRoot, cloudUID);
		ccPointCloud* cloud = foundObj ? ccHObjectCaster::ToPointCloud(foundObj) : nullptr;
		if ( !cloud )
		{
			appInterface->dispToConsole("[FFD] Original point cloud not found in DB", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return;
		}

		// Read current lattice configuration
		std::array<unsigned int, 3> oldDims = latticeDisplay->getDims();
		double oldRotation = latticeDisplay->getRotationDeg();
		int oldDeformType = latticeDisplay->getDeformType();

		// Open dialog pre-filled with current values
		FFDLatticeParamsDlg paramsDlg(nullptr);
		paramsDlg.setInitialValues(oldDims, oldDeformType, oldRotation);

		ccGLWindowInterface* win = appInterface->getActiveGLWindow();
		if (win)
		{
			paramsDlg.setPreviewContext(cloud->getOwnBB(), win);
		}

		// Collect available point clouds for trajectory selection
		{
			ccHObject::Container pointClouds;
			if (dbRoot)
				dbRoot->filterChildren(pointClouds, true, CC_TYPES::POINT_CLOUD, true);
			std::vector<ccPointCloud*> candidates;
			for (ccHObject* entity : pointClouds)
			{
				ccPointCloud* pc = ccHObjectCaster::ToPointCloud(entity);
				if (pc)
					candidates.push_back(pc);
			}
			paramsDlg.setAvailableTrajectories(candidates, cloud);
		}

		if (paramsDlg.exec() != QDialog::Accepted)
		{
			appInterface->dispToConsole("[FFD] Edit cancelled", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
			return;
		}

		// Get new parameters
		std::array<unsigned int, 3> latticeSize = paramsDlg.getLatticeSize();
		DeformationType deformType = paramsDlg.getDeformationType();
		double rotationZ = paramsDlg.getRotationZ();
		ccPointCloud* trajectoryCloud = paramsDlg.getSelectedTrajectory();

		// Remove the old FFD group (parent of lattice display)
		ccHObject* ffdGroup = latticeDisplay->getParent();

		// Create new lattice
		FFDLattice* lattice = new FFDLattice(latticeSize, cloud->getOwnBB(), rotationZ);
		lattice->setDeformationType(deformType);

		// Create new subsampled preview cloud
		size_t fullSize = cloud->size();
		size_t targetPreviewSize = paramsDlg.getPreviewPointCount();
		size_t step = (fullSize > targetPreviewSize && targetPreviewSize > 0) ? (fullSize / targetPreviewSize) : 1;
		step = std::max<size_t>(step, 1);

		CCCoreLib::ReferenceCloud previewRef(cloud);
		previewRef.reserve(static_cast<unsigned>(fullSize / step + 1));
		for (size_t i = 0; i < fullSize; i += step)
		{
			previewRef.addPointIndex(static_cast<unsigned>(i));
		}

		ccPointCloud* previewCloud = cloud->partialClone(&previewRef);
		if (!previewCloud)
		{
			previewCloud = new ccPointCloud("FFD Preview");
			previewCloud->reserve(previewRef.size());
			for (unsigned i = 0; i < previewRef.size(); ++i)
			{
				const CCVector3* P = cloud->getPoint(previewRef.getPointGlobalIndex(i));
				previewCloud->addPoint(*P);
			}
		}
		previewCloud->setName(QString("%1 (FFD Preview)").arg(cloud->getName()));
		previewCloud->setVisible(true);
		previewCloud->setEnabled(true);
		previewCloud->setPointSize(cloud->getPointSize());

		// Create new lattice display
		ccFFDLatticeDisplay* newLatticeDisplay = new ccFFDLatticeDisplay(cloud->getOwnBB(), latticeSize, lattice->getAllControlPoints());
		newLatticeDisplay->setName("FFD Lattice");
		newLatticeDisplay->setLatticeConfig(rotationZ, static_cast<int>(deformType), cloud->getUniqueID());

		// Remove old group from DB and add new one
		if (ffdGroup)
		{
			appInterface->removeFromDB(ffdGroup);
		}

		ccHObject* newGroup = new ccHObject("FFD");
		newGroup->addChild(previewCloud);
		newGroup->addChild(newLatticeDisplay);
		appInterface->addToDB(newGroup, false, true, false, true);

		// Create and setup the interactive deformation tool
		ccFFDDeformationTool* tool = new ccFFDDeformationTool(cloud, previewCloud, appInterface);
		s_activeTool = tool;

		if (trajectoryCloud)
		{
			tool->setTrajectoryCloud(trajectoryCloud);
			appInterface->dispToConsole(
				QString("[FFD] Associated trajectory: %1 (%2 points)")
					.arg(trajectoryCloud->getName())
					.arg(trajectoryCloud->size()),
				ccMainAppInterface::STD_CONSOLE_MESSAGE);
		}

		tool->setLattice(lattice, newLatticeDisplay);

		if (!win)
			win = appInterface->getActiveGLWindow();
		if (!win)
		{
			appInterface->dispToConsole("[FFD] No active 3D window", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			delete tool;
			return;
		}

		tool->linkWith(win);
		tool->start();

		QObject::connect(tool, &ccOverlayDialog::processFinished, [tool](bool)
		{
			if (s_activeTool == tool)
				s_activeTool = nullptr;
			tool->deleteLater();
		});

		appInterface->dispToConsole(
			QString("[FFD] Lattice edited: %1x%2x%3, rotation %.1f°")
				.arg(latticeSize[0]).arg(latticeSize[1]).arg(latticeSize[2]).arg(rotationZ),
			ccMainAppInterface::STD_CONSOLE_MESSAGE);

		appInterface->refreshAll();
	}
}
