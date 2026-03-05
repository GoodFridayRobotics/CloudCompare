#pragma once

#include <QDialog>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <ccBBox.h>
#include <array>
#include <vector>

class ccHObject;
class ccPointCloud;
class ccGLWindowInterface;
class ccFFDLatticeDisplay;
enum class DeformationType;

class FFDLatticeParamsDlg : public QDialog
{
	Q_OBJECT

public:
	explicit FFDLatticeParamsDlg(QWidget* parent = nullptr);
	~FFDLatticeParamsDlg() override;

	//! Set the bounding box and GL window for live lattice preview
	void setPreviewContext(const ccBBox& bbox, ccGLWindowInterface* win);

	//! Populate the trajectory combo with available point clouds from the DB tree
	void setAvailableTrajectories(const std::vector<ccPointCloud*>& clouds, ccPointCloud* excludeCloud = nullptr);

	std::array<unsigned int, 3> getLatticeSize() const;
	DeformationType getDeformationType() const;
	size_t getPreviewPointCount() const;
	//! Returns the lattice rotation around Z axis in degrees
	double getRotationZ() const;
	//! Returns the selected trajectory cloud (nullptr if "None" is selected)
	ccPointCloud* getSelectedTrajectory() const;

	//! Pre-fill dialog with existing lattice parameters (for editing)
	void setInitialValues(const std::array<unsigned int, 3>& dims, int deformType, double rotationZ);

private Q_SLOTS:
	void updateLatticePreview();

private:
	void removeLatticePreview();

	QSpinBox* m_spinX;
	QSpinBox* m_spinY;
	QSpinBox* m_spinZ;
	QComboBox* m_deformTypeCombo;
	QSpinBox* m_spinPreviewPoints;
	QDoubleSpinBox* m_spinRotationZ;
	QComboBox* m_trajectoryCombo;
	std::vector<ccPointCloud*> m_trajectories;

	// Live lattice preview
	ccBBox m_previewBBox;
	ccGLWindowInterface* m_glWindow = nullptr;
	ccFFDLatticeDisplay* m_previewDisplay = nullptr;
};
