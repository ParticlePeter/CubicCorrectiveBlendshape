

#include "SphericalCoordinates.h"
//#include "api_macros.h"



#include <maya/MGlobal.h>
#include <maya/MMatrix.h>
#include <maya/MStatus.h>
#include <maya/MVector.h>
#include <maya/MDataHandle.h>
#include <maya/MQuaternion.h>

#include <maya/MFnEnumAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnNumericAttribute.h>



#define MCHECKERROR( STAT, MSG )	\
if( MS::kSuccess != STAT ) {		\
	cerr << MSG << endl;			\
	return MS::kFailure;			\
}

#define MCHECKERRORNORET( STAT, MSG )	\
if( MS::kSuccess != STAT ) {			\
	cerr << MSG << endl;				\
}



MTypeId SphericalCoordinates::id( 0x00000003 );


MObject	SphericalCoordinates::mAttr_inputMatrix;
MObject	SphericalCoordinates::mAttr_referenceMatrix;
MObject	SphericalCoordinates::mAttr_pole;
MObject	SphericalCoordinates::mAttr_seam;
MObject	SphericalCoordinates::mAttr_rangeU;
MObject	SphericalCoordinates::mAttr_minU;
MObject	SphericalCoordinates::mAttr_maxU;
MObject	SphericalCoordinates::mAttr_rangeV;
MObject	SphericalCoordinates::mAttr_minV;
MObject	SphericalCoordinates::mAttr_maxV;
MObject	SphericalCoordinates::mAttr_param;
MObject	SphericalCoordinates::mAttr_paramU;
MObject	SphericalCoordinates::mAttr_paramV;



inline void spherical2cartesian( MVector & vec ) {
	double sinTheta = sin( vec.y );
	double cosTheta = cos( vec.y );
	vec.x = vec.z * sinTheta * cos( vec.x );
	vec.y = vec.z * sinTheta * sin( vec.x );
	vec.z = vec.z * cosTheta;
}


inline void cartesianl2spherical( MVector & vec ) {
	double r = vec.length();
	//double p = atan2( vec.y, vec.x );
	vec.x = atan2( vec.y, vec.x ) + M_PI;
	vec.y = acos( vec.z / r );
	vec.z = r;
}


// Used to define the origin of spherical coordiantes of a joint/transform
enum {
	X_POS = 0,
	Y_POS,
	Z_POS,
	X_NEG,
	Y_NEG,
	Z_NEG
};



void * SphericalCoordinates::creator( ) { return new SphericalCoordinates; }



MStatus SphericalCoordinates::initialize( ) {

	MGlobal::displayInfo( "Initializing SphericalCoordinates node !" );

	MStatus mStatus;
	MFnEnumAttribute mFnEnumAttribute;
	MFnMatrixAttribute mFnMatrixAttribute;
	MFnNumericAttribute mFnNumericAttribute;

	cerr << endl;

	// First create Output Attributes, as we must specify which inputs affect these outputs
	mAttr_paramU = mFnNumericAttribute.create( "parameterU", "u", MFnNumericData::kDouble ); MCHECKERRORNORET( mStatus, "Create parameterU" );
	mFnNumericAttribute.setStorable( false );
	mFnNumericAttribute.setWritable( true );
	mFnNumericAttribute.setChannelBox( true );
	mAttr_paramV = mFnNumericAttribute.create( "parameterV", "v", MFnNumericData::kDouble ); MCHECKERRORNORET( mStatus, "Create parameterV" );
	mFnNumericAttribute.setStorable( false );
	mFnNumericAttribute.setWritable( true );
	mFnNumericAttribute.setChannelBox( true );
	mAttr_param = mFnNumericAttribute.create( "parameter", "p", mAttr_paramU, mAttr_paramV ); MCHECKERRORNORET( mStatus, "Create parameter" );
	mFnNumericAttribute.setStorable( false );
	mFnNumericAttribute.setWritable( true );
	mFnNumericAttribute.setChannelBox( true );
	mStatus = addAttribute( mAttr_param ); MCHECKERRORNORET( mStatus, "Add Attribute parameter" );

	// Now create Input Attributes and set Affection
	mAttr_inputMatrix = mFnMatrixAttribute.create( "inputMatrix", "im" ); MCHECKERRORNORET( mStatus, "Create Input Matrix" );
	mFnMatrixAttribute.setDefault( MMatrix( ) );
	mFnMatrixAttribute.setConnectable( true );
	mFnMatrixAttribute.setStorable( false );
	mStatus = addAttribute( mAttr_inputMatrix ); MCHECKERRORNORET( mStatus, "Add Attribute inputMatrix" );
	mStatus = attributeAffects( mAttr_inputMatrix, mAttr_param ); MCHECKERRORNORET( mStatus, "Attribute inputMatrix affects parameter" );

	mAttr_referenceMatrix = mFnMatrixAttribute.create( "referenceMatrix", "rm" ); MCHECKERRORNORET( mStatus, "Create Reference Matrix" );
	mFnMatrixAttribute.setDefault( MMatrix( ) );
	mFnMatrixAttribute.setConnectable( true );
	mFnMatrixAttribute.setStorable( true );
	mStatus = addAttribute( mAttr_referenceMatrix ); MCHECKERRORNORET( mStatus, "Add Attribute referenceMatrix" );
	mStatus = attributeAffects( mAttr_referenceMatrix, mAttr_param ); MCHECKERRORNORET( mStatus, "Attribute referenceMatrix affects parameter" );

	mAttr_pole = mFnEnumAttribute.create( "poleVector", "pole", 0, &mStatus );
	mStatus = mFnEnumAttribute.addField( "X-Pos", X_POS );
	mStatus = mFnEnumAttribute.addField( "Y-Pos", Y_POS );
	mStatus = mFnEnumAttribute.addField( "Z-Pos", Z_POS );
	mStatus = mFnEnumAttribute.addField( "X-Neg", X_NEG );
	mStatus = mFnEnumAttribute.addField( "Y-Neg", Y_NEG );
	mStatus = mFnEnumAttribute.addField( "Z-Neg", Z_NEG );
	mStatus = mFnEnumAttribute.setDefault( X_POS );
	mStatus = mFnEnumAttribute.setKeyable( true  );
	mStatus = addAttribute( mAttr_pole ); MCHECKERRORNORET( mStatus, "Add Attribute poleVector" );
	mStatus = attributeAffects( mAttr_pole, mAttr_param ); MCHECKERRORNORET( mStatus, "Attribute poleVector affects parameter" );

	mAttr_seam = mFnEnumAttribute.create( "seamVector", "seam", 0, &mStatus );
	mStatus = mFnEnumAttribute.addField( "X-Pos", X_POS );
	mStatus = mFnEnumAttribute.addField( "Y-Pos", Y_POS );
	mStatus = mFnEnumAttribute.addField( "Z-Pos", Z_POS );
	mStatus = mFnEnumAttribute.addField( "X-Neg", X_NEG );
	mStatus = mFnEnumAttribute.addField( "Y-Neg", Y_NEG );
	mStatus = mFnEnumAttribute.addField( "Z-Neg", Z_NEG );
	mStatus = mFnEnumAttribute.setDefault( Y_POS );
	mStatus = mFnEnumAttribute.setKeyable( true  );
	mStatus = addAttribute( mAttr_seam ); MCHECKERRORNORET( mStatus, "Add Attribute seamVector" );
	mStatus = attributeAffects( mAttr_seam, mAttr_param ); MCHECKERRORNORET( mStatus, "Attribute seamVector affects parameter" );

	mAttr_minU = mFnNumericAttribute.create( "minimumU", "minu", MFnNumericData::kDouble ); MCHECKERRORNORET( mStatus, "Create URange Min" );
	mFnNumericAttribute.setStorable( true );
	mAttr_maxU = mFnNumericAttribute.create( "maximumU", "maxu", MFnNumericData::kDouble ); MCHECKERRORNORET( mStatus, "Create URange Max" );
	mFnNumericAttribute.setStorable( true );
	mAttr_rangeU = mFnNumericAttribute.create( "rangeU", "ru", mAttr_minU, mAttr_maxU ); MCHECKERRORNORET( mStatus, "Create URange Parent" );
	mFnNumericAttribute.setDefault( 0.0, 4.0 );
	mFnNumericAttribute.setStorable( true );
	mFnNumericAttribute.setKeyable( true );
	mStatus = addAttribute( mAttr_rangeU ); MCHECKERRORNORET( mStatus, "Add Attribute rangeU" );
	mStatus = attributeAffects( mAttr_rangeU, mAttr_param ); MCHECKERRORNORET( mStatus, "Attribute rangeU affects parameter" );

	mAttr_minV = mFnNumericAttribute.create( "minimumV", "minv", MFnNumericData::kDouble ); MCHECKERRORNORET( mStatus, "Create VRange Min" );
	mFnNumericAttribute.setStorable( true );
	mAttr_maxV = mFnNumericAttribute.create( "maximumV", "maxv", MFnNumericData::kDouble ); MCHECKERRORNORET( mStatus, "Create VRange Max" );
	mFnNumericAttribute.setStorable( true );
	mAttr_rangeV = mFnNumericAttribute.create( "rangeV", "rv", mAttr_minV, mAttr_maxV ); MCHECKERRORNORET( mStatus, "Create VRange Parent" );
	mFnNumericAttribute.setDefault( 0.0, 2.0 );
	mFnNumericAttribute.setStorable( true );
	mFnNumericAttribute.setKeyable( true );
	mStatus = addAttribute( mAttr_rangeV ); MCHECKERRORNORET( mStatus, "Add Attribute rangeV" );
	mStatus = attributeAffects( mAttr_rangeV, mAttr_param ); MCHECKERRORNORET( mStatus, "Attribute rangeV affects parameter" );

	return MS::kSuccess;
}


MStatus SphericalCoordinates::compute( const MPlug & mPlug, MDataBlock & mDataBlock ) {

	MStatus mStatus;

	if(( mPlug == mAttr_param ) || ( mPlug.parent( ) == mAttr_param ) ) {

		// Get the seam and pole axis
		short pole = mDataBlock.inputValue( mAttr_pole ).asShort( );
		short seam = mDataBlock.inputValue( mAttr_seam ).asShort( );

		if( pole == seam )
			return MS::kFailure;

		// Negative Axis are the last 3 Enum Values
		double dirPole = pole < 3 ? 1.0 : -1.0;
		double dirSeam = seam < 3 ? -1.0 : 1.0;		// Now Implementing Wikipedia Spherical Coordinates, have to fix seam

		pole = pole % 3;
		seam = seam % 3;
		short last = ( 2 * ( pole + seam ) ) % 3;

		// Get the Data Handle to the Input Matrix and Extract its Contents into a Pole Vector
		MMatrix mInputMatrix = mDataBlock.inputValue( mAttr_inputMatrix, &mStatus ).asMatrix( );
		MVector mVecPoleInput( mInputMatrix( pole, 0 ), mInputMatrix( pole, 1 ), mInputMatrix( pole, 2 ) );
		mVecPoleInput = dirPole * mVecPoleInput;
		mVecPoleInput.normalize( );


		// Get the Data Handle to the Reference Matrix and Extract its Contents into a Pole Vector
		MMatrix mReferenceMatrix = mDataBlock.inputValue( mAttr_referenceMatrix, &mStatus ).asMatrix( );
		MVector mPoleReference( mReferenceMatrix( pole, 0 ), mReferenceMatrix( pole, 1 ), mReferenceMatrix( pole, 2 ) );
		mPoleReference = dirPole * mPoleReference;
		mPoleReference.normalize( );

		// Multiply Vec Pole from Input Matrix with Inverse ( Transpose ) of Reference Matrix
		MVector mVecPT = mVecPoleInput * mReferenceMatrix.transpose();

		// Create a Rotation Matrix such that Pole points into Z Direction and Seam points into X
		MMatrix mSphereMatrix;
		mSphereMatrix[0][0] = mSphereMatrix[1][1] = mSphereMatrix[2][2] = 0;
		mSphereMatrix[seam][0] = dirSeam;
		mSphereMatrix[last][1] = dirPole * dirSeam;
		mSphereMatrix[pole][2] = dirPole;

		// Multiply Result with New created Matrix Rotation Matrix --- for Vertices
		mVecPT *= mSphereMatrix;

		// Use cartesian2sperical to get Phi and Theta
		cartesianl2spherical( mVecPT );

		// Get the range for param v and apply to Spehrical Coordinates Theta
		double * range = mDataBlock.inputValue( mAttr_rangeV ).asDouble2( );
		double rangeDelta = range[1] - range[0];
		double valV = range[0] + rangeDelta * mVecPT.y / M_PI;

		// Get the range for param U and apply to Spehrical Coordinates Phi
		range = mDataBlock.inputValue( mAttr_rangeU ).asDouble2( );
		rangeDelta = range[1] - range[0];
		double valU = range[0] + rangeDelta * 0.5 * mVecPT.x / M_PI;

		// Get a handle to Parameter V Attribute, which is a pure outputand set its VAlue
		MDataHandle mParamV = mDataBlock.outputValue( mAttr_paramV );
		mParamV.set( valV );

		// Get a handle to Parameter V Attribute, which is a pure outputand set its VAlue
		MDataHandle mParamU = mDataBlock.outputValue( mAttr_paramU );
		mParamU.set( valU );

		// Setting the MPlug as clean prevents recalcualtion until an Input has changed
		mDataBlock.setClean( mPlug );

	} else {
		return MS::kUnknownParameter;
	}

	return MS::kSuccess;
}

/*
MStatus initializePlugin( MObject mObject ) {
MGlobal::displayInfo( "Initializing SphericalCoordinates PlugIn !" );
MStatus mStatus;
MFnPlugin plugin( mObject, "Peter Particle", "1.0", "Any" );

// Specify we are making a deformer node
mStatus = plugin.registerNode( "SphericalCoordinates", SphericalCoordinates::id, SphericalCoordinates::creator, SphericalCoordinates::initialize, MPxNode::kDeformerNode );
CHECK_MSTATUS_AND_RETURN_IT( mStatus );

return mStatus;
}


MStatus uninitializePlugin( MObject mObject ) {
MStatus		mStatus;
MFnPlugin	plugin( mObject );

mStatus = plugin.deregisterNode( SphericalCoordinates::id );
CHECK_MSTATUS_AND_RETURN_IT( mStatus );

return mStatus;
}*/