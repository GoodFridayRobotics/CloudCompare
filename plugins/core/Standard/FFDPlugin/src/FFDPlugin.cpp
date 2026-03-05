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

#include "FFDPlugin.h"
#include "FFDAction.h"
#include "ccFFDLatticeDisplay.h"

#include <QtGui>

FFDPlugin::FFDPlugin( QObject *parent )
	: QObject( parent )
	, ccStdPluginInterface( ":/CC/plugin/FFDPlugin/info.json" )
	, m_deformAction( nullptr )
	, m_editLatticeAction( nullptr )
{
	createAction();
}

void FFDPlugin::createAction()
{
	if ( m_deformAction == nullptr )
	{
		m_deformAction = new QAction( "Free Form Deformation", this );
		m_deformAction->setStatusTip( "Apply non-rigid deformation to point cloud using a lattice" );
		// Icon path will be set once an icon.png is added to the images/ directory
		
		connect( m_deformAction, &QAction::triggered, this, [this]()
		{
			// Forward to the action handler
			FFDAction::performDeformation( m_app );
		} );
	}

	if ( m_editLatticeAction == nullptr )
	{
		m_editLatticeAction = new QAction( "Edit FFD Lattice", this );
		m_editLatticeAction->setStatusTip( "Edit the selected FFD lattice parameters" );

		connect( m_editLatticeAction, &QAction::triggered, this, [this]()
		{
			FFDAction::editLattice( m_app );
		} );
	}
}

void FFDPlugin::onNewSelection( const ccHObject::Container &selectedEntities )
{
	if ( m_deformAction == nullptr )
	{
		return;
	}

	// Enable the action only if a point cloud is selected
	bool hasPointCloud = false;

	for ( ccHObject *entity : selectedEntities )
	{
		if ( entity->isA( CC_TYPES::POINT_CLOUD ) || entity->isA( CC_TYPES::MESH ) )
		{
			hasPointCloud = true;
			break;
		}
	}

	m_deformAction->setEnabled( hasPointCloud );
}

QList<QAction*> FFDPlugin::getActions()
{
	return { m_deformAction };
}

QList<QAction*> FFDPlugin::getEntityContextMenuActions(const ccHObject::Container &selectedEntities)
{
	QList<QAction*> actions;

	for ( ccHObject *entity : selectedEntities )
	{
		if ( ccFFDLatticeDisplay::IsFFDLatticeDisplay(entity) )
		{
			actions << m_editLatticeAction;
			break;
		}
	}

	return actions;
}
