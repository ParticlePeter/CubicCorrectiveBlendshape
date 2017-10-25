#ifndef CubicBlendNode_H
#define CubicBlendNode_H


#include <maya/MVector.h>
#include <maya/MMatrix.h>
#include <maya/MDataBlock.h>
#include <maya/MItGeometry.h>
#include <maya/MObjectArray.h>
#include <maya/MPxDeformerNode.h>



class CubicBlendNode : public MPxDeformerNode {
public :
	CubicBlendNode() {};
	virtual MStatus deform( MDataBlock & mDataBlock, MItGeometry & mItGeometry, const MMatrix & mMatrix_localToWorld, unsigned int multiIndex );
	static void * creator();
	static MStatus initialize();

	static MTypeId id;

	// For Spherical Coordinates
	static MObject mAttr_inputMatrix;		// source matrix for sperical coordinates
	static MObject mAttr_referenceMatrix;	// reference matrix to compute sph. coords. from input matrix
	static MObject mAttr_pole;				// pole (axis) direction of speherical coordinates spehere in local reference matrix space 
	static MObject mAttr_seam;				// seam of spherical coordinates sphere
	static MObject mAttr_param;				// Azimuth and polar angles are expressed as UV-Parameters similar a NURBs Sphere
	static MObject		mAttr_paramU;		// azimuth angle phi
	static MObject		mAttr_paramV;		// polar angle theta

	// For Deformer
	static MObject mAttr_targetShapes;		// 
	static MObject mAttr_interpolator;

};
#endif