#pragma once

#include <ccCustomObject.h>
#include <ccBBox.h>
#include <CCGeom.h>

#include <array>
#include <vector>

class ccFFDLatticeDisplay : public ccCustomHObject
{
public:
	ccFFDLatticeDisplay(const ccBBox& bbox,
	                   const std::array<unsigned int, 3>& dims,
	                   const std::vector<CCVector3d>& controlPoints);

	ccBBox getOwnBB(bool withGLFeatures = false) override;

	void setControlPoints(const std::vector<CCVector3d>& controlPoints);
	void setSelectedIndices(const std::vector<int>& indices);

	//! Store lattice configuration for later editing
	void setLatticeConfig(double rotationDeg, int deformType, unsigned int originalCloudUniqueID);

	//! Get lattice dimensions
	const std::array<unsigned int, 3>& getDims() const { return m_dims; }
	//! Get rotation angle in degrees
	double getRotationDeg() const { return m_rotationDeg; }
	//! Get deformation type (0=Linear, 1=BSpline)
	int getDeformType() const { return m_deformType; }
	//! Get the unique ID of the original cloud
	unsigned int getOriginalCloudUniqueID() const { return m_originalCloudUniqueID; }

	//! Returns true if the given entity is an FFD lattice display
	static bool IsFFDLatticeDisplay(const ccHObject* entity);

	//! Metadata key used to tag FFD lattice display entities
	static constexpr const char* MetaDataClassName() { return "FFDLatticeDisplay"; }

protected:
	void drawMeOnly(CC_DRAW_CONTEXT& context) override;

private:
	void updateBoundingBox();
	const CCVector3d& getControlPoint(unsigned int x, unsigned int y, unsigned int z) const;

	ccBBox m_bbox;
	std::array<unsigned int, 3> m_dims;
	std::vector<CCVector3d> m_controlPoints;
	std::vector<int> m_selectedIndices;

	// Lattice configuration for editing
	double m_rotationDeg = 0.0;
	int m_deformType = 1; // BSpline
	unsigned int m_originalCloudUniqueID = 0;
};
