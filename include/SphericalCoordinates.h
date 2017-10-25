#ifndef SPHERICAL_COORDINATES_H
#define SPHERICAL_COORDINATES_H



#include <maya/MMatrix.h>
#include <maya/MDataBlock.h>
#include <maya/MItGeometry.h>
#include <maya/MObjectArray.h>
#include <maya/MPxDeformerNode.h>



class SphericalCoordinates : public MPxNode{
public :
	SphericalCoordinates( ) {};
	virtual MStatus compute( const MPlug& plug, MDataBlock& data );
	static void * creator();
	static MStatus initialize();

	static MTypeId id;
	static MObject mAttr_inputMatrix;
	static MObject mAttr_referenceMatrix;
	static MObject mAttr_pole;
	static MObject mAttr_seam;
	static MObject mAttr_rangeU;
	static MObject		mAttr_minU;
	static MObject		mAttr_maxU;
	static MObject mAttr_rangeV;
	static MObject		mAttr_minV;
	static MObject		mAttr_maxV;
	static MObject mAttr_param;
	static MObject		mAttr_paramU;
	static MObject		mAttr_paramV;
	//static MObject mUDirection;
	//static MObject mVDirection;


};
#endif

/*
MStatus buildRotation::initialize()
{
MFnNumericAttribute nAttr;
MFnMatrixAttribute mAttr;
MFnUnitAttribute uAttr;
MFnEnumAttribute eAttr;
MStatus stat;
// Set up inputs
//
upX = nAttr.create( "upX", "ux", MFnNumericData::kDouble );
nAttr.setStorable(false);
upY = nAttr.create( "upY", "uy", MFnNumericData::kDouble );
nAttr.setStorable(false);
upZ = nAttr.create( "upZ", "uz", MFnNumericData::kDouble );
nAttr.setStorable(false);
up = nAttr.create( "up", "u", upX, upY, upZ );
nAttr.setDefault( 0.0, 1.0, 0.0 );
nAttr.setStorable(false);
forwardX = nAttr.create( "forwardX", "fx", MFnNumericData::kDouble, 0.0 );
nAttr.setStorable(false);
forwardY = nAttr.create( "forwardY", "fy", MFnNumericData::kDouble, 0.0 );
nAttr.setStorable(false);
forwardZ = nAttr.create( "forwardZ", "fz", MFnNumericData::kDouble, 1.0 );
nAttr.setStorable(false);
forward = nAttr.create( "forward", "f", forwardX, forwardY, forwardZ );
nAttr.setDefault( 0.0, 0.0, 1.0 );
nAttr.setStorable(false);
rotateOrder = eAttr.create( "rotateOrder", "ro", ROTATE_ORDER_XYZ);
eAttr.addField("xyz", ROTATE_ORDER_XYZ);
eAttr.addField("yzx", ROTATE_ORDER_YZX);
eAttr.addField("zxy", ROTATE_ORDER_ZXY);
eAttr.addField("xzy", ROTATE_ORDER_XZY);
eAttr.addField("yxz", ROTATE_ORDER_YXZ);
eAttr.addField("zyx", ROTATE_ORDER_ZYX);
eAttr.setStorable(false);
// Set up outputs
//
rotateX = uAttr.create( "rotateX", "rx", MFnUnitAttribute::kAngle, 0.0 );
nAttr.setStorable(false);
rotateY = uAttr.create( "rotateY", "ry", MFnUnitAttribute::kAngle, 0.0 );
nAttr.setStorable(false);
rotateZ = uAttr.create( "rotateZ", "rz", MFnUnitAttribute::kAngle, 0.0 );
nAttr.setStorable(false);
rotate = nAttr.create( "rotate", "r", rotateX, rotateY, rotateZ );
nAttr.setStorable(false);
rotateMatrix = mAttr.create( "rotateMatrix", "rm" );
nAttr.setStorable(false);
nAttr.setConnectable(true);
stat = addAttribute( up );
if (!stat) { stat.perror("addAttribute"); return stat;}
stat = addAttribute( forward );
if (!stat) { stat.perror("addAttribute"); return stat;}
stat = addAttribute( rotate );
if (!stat) { stat.perror("addAttribute"); return stat;}
stat = addAttribute( rotateOrder );
if (!stat) { stat.perror("addAttribute"); return stat;}
stat = addAttribute( rotateMatrix );
if (!stat) { stat.perror("addAttribute"); return stat;}
stat = attributeAffects( up, rotate );
if (!stat) { stat.perror("attributeAffects"); return stat;}
stat = attributeAffects( up, rotateMatrix );
if (!stat) { stat.perror("attributeAffects"); return stat;}
stat = attributeAffects( forward, rotate );
if (!stat) { stat.perror("attributeAffects"); return stat;}
stat = attributeAffects( forward, rotateMatrix );
if (!stat) { stat.perror("attributeAffects"); return stat;}
stat = attributeAffects( rotateOrder, rotate );
if (!stat) { stat.perror("attributeAffects"); return stat;}
stat = attributeAffects( rotateOrder, rotateMatrix );
if (!stat) { stat.perror("attributeAffects"); return stat;}
return MS::kSuccess;
}
*/