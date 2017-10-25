

#include "CubicBlendNode.h"
//#include "api_macros.h"



#include <maya/MFn.h>
#include <maya/MPoint.h>
#include <maya/MFnMesh.h>
#include <maya/MGlobal.h>
#include <maya/MStatus.h>
#include <maya/MVector.h>
#include <maya/MDataHandle.h>
#include <maya/MPointArray.h>
#include <maya/MQuaternion.h>
#include <maya/MFnNurbsCurve.h>
#include <maya/MFnNurbsCurveData.h>

#include <maya/MFnEnumAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnNumericAttribute.h>



#include <algorithm>

#include <iostream>


#define MCHECKERROR( STAT, MSG )  \
if( MS::kSuccess != STAT ) {      \
	cerr << MSG << endl;          \
	return MS::kFailure;          \
}

#define MCHECKERRORNORET( STAT, MSG )  \
if( MS::kSuccess != STAT ) {           \
	cerr << MSG << endl;               \
}



MTypeId CubicBlendNode::id( 0x00000002 );

// For CIRCULAR Coordiantes
MObject	CubicBlendNode::mAttr_inputMatrix;
MObject	CubicBlendNode::mAttr_referenceMatrix;
MObject	CubicBlendNode::mAttr_pole;
MObject	CubicBlendNode::mAttr_seam;
MObject	CubicBlendNode::mAttr_param;
MObject	CubicBlendNode::mAttr_paramU;
MObject	CubicBlendNode::mAttr_paramV;

MObject CubicBlendNode::mAttr_targetShapes;
MObject CubicBlendNode::mAttr_interpolator;

static MMatrix matA_inverted;


enum {
	LINEAR = 0,
	B_SPLINE,
	CIRCULAR,
	C_SPLINE,
	M_NURBS,
	C_TEMP,
};


enum {
	X_POS = 0,
	Y_POS,
	Z_POS,
	X_NEG,
	Y_NEG,
	Z_NEG
};



void * CubicBlendNode::creator() {  return new CubicBlendNode;  }


inline double triClamp( double t ) {
	return ( t < 0.0f ? 0.0f : ( t > 2.0f ? 0.0f : ( t < 1.0f ? t : 2.0f - t ) ) );
}


inline void cartesian2spherical( MVector & vec ) {
	double r = vec.length();  // Distance is irrelevant
	//double p = atan2( vec.y, vec.x );
	vec.x = atan2( vec.y, vec.x ) + M_PI;
	vec.y = acos( vec.z / r );
	vec.z = r;
}


inline void spherical2cartesian( MVector & vec ) {
	double sinTheta = sin( vec.y );
	double cosTheta = cos( vec.y );
	double sinPhi = sin( vec.x - M_PI );
	double cosPhi = cos( vec.x - M_PI );
	vec.x = vec.z * sinTheta * cosPhi;
	vec.y = vec.z * sinTheta * sinPhi;
	vec.z = vec.z * cosTheta;
}


inline void cartesian2cylindrical( MVector & vec ) {
	// transform to:
	// vec.x = r	: polar distance
	// vec.y = phi	: polar angle
	// vec.z = z	: hight component of cylinder, does not have to change at all

	// following special case must become - PI
	//if( vec.x == 0 && vec.y > 0 ) {
	//	vec.x = vec.y;
	//	vec.y = M_PI;
	//	return;
	//}
	double phi = atan2( vec.y, vec.x );	// + M_PI;
	vec.x = sqrt( vec.x * vec.x + vec.y * vec.y );
	vec.y = phi;

}


inline void cylindrical2cartesian( MVector & vec ) {
	double  vec_x = vec.x;
	vec.x = vec_x * cos( vec.y );	// - M_PI );
	vec.y = vec_x * sin( vec.y );	// - M_PI );
}


inline void computeCSpline(
	MMatrix & mReferenceMatrix,
	double paramU,
	double paramV,
	MItGeometry & mItGeometry,
	MPointArray * mPointArrays,
	bool toCartesian
	) {

	const int tmp_start = 106;
	const int tmp_count = 4;
	MVector mc_0[tmp_count];
	MVector mc_1[tmp_count];
	MVector mc_2[tmp_count];
	MVector mc_3[tmp_count];
	MVector mc_4[tmp_count];
	MVector md_0[tmp_count];
	MVector md_1[tmp_count];
	MVector md_2[tmp_count];
	MVector md_3[tmp_count];
	MVector md_4[tmp_count];
	MPoint  df_0[tmp_count];
	MPoint  df_1[tmp_count];
	MPoint  df_2[tmp_count];


	unsigned int intParamU = std::min(( int )( paramU ), 3 );	// Must not become 4
	unsigned int intParamU_1 = ( intParamU + 1 ) % 4;

	double t1 = paramU - intParamU;
	double t2 = t1 * t1;
	double t3 = t2 * t1;

	MVector mVecSphere;
	MPointArray mDifference( 3 ), mD( 3 );
	MVectorArray mDeltas( 6 );
	MVectorArray mDeltasCart( 5 );

	for( ; !mItGeometry.isDone(); mItGeometry.next() ) {

		// Get the painted paramV value
		//double vertexWeight = weightValue( mDataBlock, multiIndex, mItGeometry.index() );

		// Result
		MVector mResult;

		// Test if cartesian2... and ...2cartesian work properly
		//mResult = MVector( mItGeometry.position());
		//cartesian2spherical( mResult );
		//spherical2cartesian( mResult );
		//cartesian2cylindrical( mResult );
		//cylindrical2cartesian( mResult );
		//mItGeometry.setPosition( mResult );
		//continue;



		// 
		//mDeltasCart[0] = mPointArrays[0][ mItGeometry.index() ] - mItGeometry.position();		// Create delta shape
		//mDeltas[0] = mDeltasCart[0] * mReferenceMatrix;											// Transforme into refernece matrix space
		//cartesian2cylindrical( mDeltas[0] );													// Convert to cylindrical coordinates
		//mDeltas[0].y = fmod( mDeltas[0].y + 4 * M_PI, 2 * M_PI );								// bring into positive phi range

		// Compute delta shape between deformed orignal shape and each target shape
		// Transform each delta shape into refernce matrix space
		// Rotat deltas from second to last quadrant into the first quadrant, so that all deltas point into similar
		for( int i = 0; i < 4; ++i ) {
			mDeltasCart[i] = mPointArrays[i][ mItGeometry.index() ] - mItGeometry.position();		// Create delta shape
			mDeltas[i] = mDeltasCart[i] * mReferenceMatrix;											// Transforme into refernece matrix space
			cartesian2cylindrical( mDeltas[i] );													// Convert to cylindrical coordinates
			mDeltas[i].y += 2 * M_PI;
		}


		mDeltas[4] = mDeltas[0];
		if( mDeltas[0].y >  2 * M_PI )
			mDeltas[0].y -= 2 * M_PI;
		else
			mDeltas[4].y += 2 * M_PI;

		{
			auto mDelta_A = mDeltas[ 1 ];
			auto mDelta_B = mDeltas[ 3 ];	// -1
			//mDelta_A.y -= 2 * M_PI;
			mDelta_B.y -= 2 * M_PI;
			mResult = 3 * ( mDelta_A - mDelta_B );
			mDifference[0][0] = mResult.x;
			mDifference[1][0] = mResult.y;
			mDifference[2][0] = mResult.z;
		}

		{
			auto mDelta_A = mDeltas[ 2 ];
			auto mDelta_B = mDeltas[ 0 ];
			//mDelta_A.y += 2 * M_PI;
			//mDelta_B.y -= 2 * M_PI;
			mResult = 3 * ( mDelta_A - mDelta_B );
			mDifference[0][1] = mResult.x;
			mDifference[1][1] = mResult.y;
			mDifference[2][1] = mResult.z;
		}

		{
			auto mDelta_A = mDeltas[ 3 ];
			auto mDelta_B = mDeltas[ 1 ];
			//mDelta_A.y -= 2 * M_PI;
			//mDelta_B.y -= 2 * M_PI;
			mResult = 3 * ( mDelta_A - mDelta_B );
			mDifference[0][2] = mResult.x;
			mDifference[1][2] = mResult.y;
			mDifference[2][2] = mResult.z;
		}

		{
			auto mDelta_A = mDeltas[ 4 ];
			auto mDelta_B = mDeltas[ 2 ];
			//mDelta_A.y -= 2 * M_PI;
			//mDelta_B.y += 2 * M_PI;
			mResult = 3 * ( mDelta_A - mDelta_B );
			mDifference[0][3] = mResult.x;
			mDifference[1][3] = mResult.y;
			mDifference[2][3] = mResult.z;
		}

		auto idx = mItGeometry.index();
		if( idx >= tmp_start && idx < tmp_start + tmp_count ) {
			auto ti = idx - tmp_start;
			mc_0[ti] = mDeltasCart[0];
			mc_1[ti] = mDeltasCart[1];
			mc_2[ti] = mDeltasCart[2]; 
			mc_3[ti] = mDeltasCart[3];
			mc_4[ti] = mDeltasCart[4];
			md_0[ti] = mDeltas[0];
			md_1[ti] = mDeltas[1];
			md_2[ti] = mDeltas[2];
			md_3[ti] = mDeltas[3];
			md_4[ti] = mDeltas[4];
			df_0[ti] = mDifference[0];
			df_1[ti] = mDifference[1];
			df_2[ti] = mDifference[2];

		}

		// Compute cofficiencies
		for( int i = 0; i < 3; ++i ) {
			mD[i] = mDifference[i] * matA_inverted;
		}

		MVector a = mDeltas[ intParamU ];
		MVector a_1 = mDeltas[ intParamU + 1 ];
		MVector b( mD[0][ intParamU ], mD[1][ intParamU ], mD[2][ intParamU ] );
		MVector b_1( mD[0][ intParamU_1 ], mD[1][ intParamU_1 ], mD[2][ intParamU_1 ] );
		MVector c = 3 * ( a_1 - a ) - 2 * b - b_1;
		MVector d = 2 * ( a - a_1 ) +     b + b_1;

		// Compute point on spline using cofficiencies
		mResult = a + b * t1 + c * t2 + d * t3;
		
		if( toCartesian )
			cylindrical2cartesian( mResult );

		mResult = paramV * ( mReferenceMatrix * mResult ) + mItGeometry.position();

		// Set the new output point
		mItGeometry.setPosition( mResult );

	}
}



MStatus CubicBlendNode::initialize() {

	MGlobal::displayInfo( "Initializing CubicBlendNode node !" );

	// We need to solve Ax = b linear euations according to:
	// Bartels et.al. 1995 - An Introduction to Splines for Use in Computer Graphics and Geometric Modeling, p. 17ff
	// to interpolate 4 points in a periodic fassion we use following diagonal matrix A
	// | 4 1 0 1 |
	// | 1 4 1 0 |
	// | 0 1 4 1 |
	// | 1 0 1 4 |
	// b is known, it is the difference of the neigboring targets S_{i-1} - S_{i+1} of the currently interpolated targets S_i
	// hence we invert A to get A' and solve x = A'b
	// Note: For targte shape count N we required an NxN (inverted) matrix.
	// Here we use only a 4x4 matrix as a proof of concept
	// Hence the current implementation will work with 4 targte shapes (with mode B-Spline and C-Spline).
	for( unsigned int i = 0; i < 4; ++i ) {
		matA_inverted[ i ][ ( i + 0 ) % 4 ] =  0.2917;
		matA_inverted[ i ][ ( i + 1 ) % 4 ] = -0.0833;
		matA_inverted[ i ][ ( i + 2 ) % 4 ] =  0.0417;
		matA_inverted[ i ][ ( i + 3 ) % 4 ] = -0.0833;
	}



	MStatus mStatus;
	MFnEnumAttribute mFnEnumAttribute;
	MFnTypedAttribute mFnTypedAttribute;
	MFnMatrixAttribute mFnMatrixAttribute;
	MFnNumericAttribute mFnNumericAttribute;

	// For CIRCULAR Coordinates
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
	mStatus = attributeAffects( mAttr_inputMatrix, outputGeom  ); MCHECKERRORNORET( mStatus, "Attribute inputMatrix affects outputGeom" );

	mAttr_referenceMatrix = mFnMatrixAttribute.create( "referenceMatrix", "rm" ); MCHECKERRORNORET( mStatus, "Create Reference Matrix" );
	mFnMatrixAttribute.setDefault( MMatrix( ) );
	mFnMatrixAttribute.setConnectable( true );
	mFnMatrixAttribute.setStorable( true );
	mStatus = addAttribute( mAttr_referenceMatrix ); MCHECKERRORNORET( mStatus, "Add Attribute referenceMatrix" );
	mStatus = attributeAffects( mAttr_referenceMatrix, mAttr_param ); MCHECKERRORNORET( mStatus, "Attribute referenceMatrix affects parameter" );
	mStatus = attributeAffects( mAttr_referenceMatrix, outputGeom  ); MCHECKERRORNORET( mStatus, "Attribute referenceMatrix affects outputGeom" );

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
	mStatus = attributeAffects( mAttr_pole, outputGeom  ); MCHECKERRORNORET( mStatus, "Attribute poleVector affects outputGeom" );

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
	mStatus = attributeAffects( mAttr_seam, outputGeom  ); MCHECKERRORNORET( mStatus, "Attribute seamVector affects outputGeom" );


	// For Deformer
	mAttr_targetShapes = mFnTypedAttribute.create( "targetShape", "targetShape", MFnData::kMesh, &mStatus ); MCHECKERRORNORET( mStatus, "Create inputGeometry" );
	mStatus = mFnTypedAttribute.setArray( true ); MCHECKERRORNORET( mStatus, "Set as array inputGeometry" );
	mStatus = mFnTypedAttribute.setDisconnectBehavior( MFnAttribute::kDelete ); MCHECKERRORNORET( mStatus, "Set disconnect behavior inputGeometry" );
	mStatus = addAttribute( mAttr_targetShapes ); MCHECKERRORNORET( mStatus, "Add Attribute to Node inputGeometry" );
	mStatus = attributeAffects( mAttr_targetShapes, outputGeom ); MCHECKERRORNORET( mStatus, "Affect outputGeom with inputGeometry" );

	mAttr_interpolator = mFnEnumAttribute.create( "interpolator", "i", 0, &mStatus );
	mStatus = mFnEnumAttribute.addField( "Linear"  , LINEAR );
	mStatus = mFnEnumAttribute.addField( "B-Spline", B_SPLINE );
	mStatus = mFnEnumAttribute.addField( "Circular", CIRCULAR );
	mStatus = mFnEnumAttribute.addField( "C-Spline", C_SPLINE );
	mStatus = mFnEnumAttribute.addField( "M-NURBs" , M_NURBS );
	mStatus = mFnEnumAttribute.addField( "C-Temp"  , C_TEMP );
	mStatus = mFnEnumAttribute.setChannelBox( true );
	addAttribute( mAttr_interpolator );
	mStatus = attributeAffects( mAttr_interpolator, outputGeom ); MCHECKERRORNORET( mStatus, "Attribute interpolator affects outputGeom" );

	return MS::kSuccess;
}


MStatus CubicBlendNode::deform( MDataBlock & mDataBlock, MItGeometry & mItGeometry, const MMatrix & mMatrix_localToWorld, unsigned int multiIndex ) {

	MStatus mStatus;

	// --- First Determine Phi and Theta --- //

	// Get the seam and pole axis
	short pole = mDataBlock.inputValue( mAttr_pole ).asShort( );
	short seam = mDataBlock.inputValue( mAttr_seam ).asShort( );

	if( pole == seam ) {
		cerr << "Pole and Seam must not be parallel !";
		return MS::kFailure;
	}

	// Negative Axis are the last 3 Enum Values
	double dirPole = pole < 3 ? 1.0 : -1.0;
	double dirSeam = seam < 3 ? -1.0 : 1.0;		// Now Implementing Wikipedia CIRCULAR Coordinates, have to fix seam

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



	// Create a Rotation Matrix such that Pole points into Z Direction and Seam points into X
	MMatrix mMat_Z_Up;
	mMat_Z_Up[0][0] = mMat_Z_Up[1][1] = mMat_Z_Up[2][2] = 0;
	mMat_Z_Up[seam][0] = dirSeam;
	mMat_Z_Up[last][1] = dirPole * dirSeam;
	mMat_Z_Up[pole][2] = dirPole;

	mReferenceMatrix = mReferenceMatrix.transpose() * mMat_Z_Up;


	// Multiply Result with New created Matrix Rotation Matrix --- for Vertices
	mVecPoleInput *= mReferenceMatrix;

	// Use cartesian2sperical to get Phi and Theta
	cartesian2spherical( mVecPoleInput );

	double phi = mVecPoleInput.x;
	double theta = mVecPoleInput.y;

	// These Attributes are meant as output, for debug purpose
	MDataHandle mParamU = mDataBlock.outputValue( mAttr_paramU ); mParamU.set( phi );
	MDataHandle mParamV = mDataBlock.outputValue( mAttr_paramV ); mParamV.set( theta );



	// --- Now Deform --- use Phi and Theta //

	// Get the Data Handle to the Input Geometries Array, return if Array has 0 length

	MArrayDataHandle mArrayGeometryHandle = mDataBlock.outputArrayValue( mAttr_targetShapes, &mStatus );
	int numGeometries = mArrayGeometryHandle.elementCount();
	if( numGeometries < 1 )  return MStatus::kSuccess;

	// Get Data Points of all connected input Geometries
	MPointArray * mPointArrays = new MPointArray[ numGeometries ];
	mArrayGeometryHandle.jumpToElement( 0 );
	numGeometries = 0;


	// Count the connected Gemetries ( sparse array ! ) and get the Vertices as an array of MPointArrays

	do {
		MObject mInputGeom( mArrayGeometryHandle.inputValue().asMesh() );

		// Get the source points
		MFnMesh mFnMesh( mInputGeom, & mStatus );
		//CHECK_MSTATUS_AND_RETURN_IT( mStatus );
		mFnMesh.getPoints( mPointArrays[ numGeometries ] );
		++numGeometries;

	} while( mArrayGeometryHandle.next() == MS::kSuccess );

	// Get the U and V Values, Interpolator and Default Envelope Parameter
	//MDataHandle mEnvelopeHandle = mDataBlock.inputValue( envelope );
	//double envelo = mEnvelopeHandle.asFloat();
	double envelo = mDataBlock.inputValue( envelope ).asFloat();
	short  interp = mDataBlock.inputValue( mAttr_interpolator ).asShort();

	// Get Parameter Values
	double * mParam = mDataBlock.inputValue( mAttr_param ).asDouble2();
	double paramU = 2 * phi / M_PI;			// remap Range from [0..2PI) to [0..4)
	double paramV = 2 * theta / M_PI;		// remap Range from [0.. PI) to [0..2)

	paramU *= numGeometries * 0.25;
	paramV *= envelo;

	//double factParamU = numGeometries / paramU;
	double rangeU = ( double )numGeometries;
	double invPaV = 1.0f - paramV;

	// Linear deformation
	if( interp == LINEAR ) {

		for( ; !mItGeometry.isDone(); mItGeometry.next() ) {

			// Use this function to get painted deformer weights
			//double vertexWeight = weightValue( mDataBlock, multiIndex, mItGeometry.index() );

			// Result
			MPoint mPoint;

			for( int i = 0; i < numGeometries; ++i )
				mPoint += triClamp( fmod( paramU + rangeU + 1.0f - i, rangeU ) ) * ( mPointArrays[ i ][ mItGeometry.index() ] ); // * vertexWeight

			mPoint = paramV * mPoint + invPaV * mItGeometry.position();

			// Set the new output point
			mItGeometry.setPosition( mPoint );
		}
	}


	// B-Spline Deformation
	else if( interp == B_SPLINE ) {

		MMatrix mReferTransM = mReferenceMatrix.transpose();

		unsigned int intParamU = std::min(( int )( paramU ), 3 );	// Must not become 4
		unsigned int intParamU_1 = ( intParamU + 1 ) % 4;

		double t1 = paramU - intParamU;
		double t2 = t1 * t1;
		double t3 = t2 * t1;

		MVector mVecSphere;
		MPointArray mDifference( 3 ), mD( 3 );
		MVectorArray mDeltas( 4 );


		for( ; !mItGeometry.isDone(); mItGeometry.next() ) {

			// Get the painted paramV value
			//double vertexWeight = weightValue( mDataBlock, multiIndex, mItGeometry.index() );

			// Result
			MVector mResult;

			// Compute delta shape between deformed orignal shape and each target shape
			// Transform each delta shape into refernce matrix space
			for( int i = 0; i < 4; ++i ) {
				mDeltas[i] = mPointArrays[ i ][ mItGeometry.index() ] - mItGeometry.position();		// Create delta shape
				mDeltas[i] *= mReferenceMatrix;														// Transforme into refernece matrix space
			}

			// From equation Ax = b compute b with b = S_{i-1} - S_{i+1}
			for( int i = 0; i < 4; ++i ) {
				mResult = 3 * ( mDeltas[ ( i + 1 + 4 ) % 4 ] - mDeltas[ ( i - 1 + 4 ) % 4 ] );
				mDifference[0][i] = mResult.x;
				mDifference[1][i] = mResult.y;
				mDifference[2][i] = mResult.z;
			}

			// Apply Bartels inverted conversion matrix
			for( int i = 0; i < 3; ++i ) {
				mD[i] = mDifference[i] * matA_inverted;
			}

			// Compute Coefficiencies
			MVector a = mDeltas[ intParamU ];
			MVector a_1 = mDeltas[ intParamU_1 ];
			MVector b( mD[0][ intParamU ], mD[1][ intParamU ], mD[2][ intParamU ] );
			MVector b_1( mD[0][ intParamU_1 ], mD[1][ intParamU_1 ], mD[2][ intParamU_1 ] );
			MVector c = 3 * ( a_1 - a ) - 2 * b - b_1;
			MVector d = 2 * ( a - a_1 ) +     b + b_1;

			mResult = a + b * t1 + c * t2 + d * t3;

			//mResult = mResult.rotateBy( MVector::kZaxis, phi );
			mResult *= mReferTransM;

			mResult = paramV * mResult + mItGeometry.position();

			// Set the new output point
			mItGeometry.setPosition( mResult );
		}
	}

	// Linear interpolation in (quasi-) cylindrical coordinate space
	else if( interp == CIRCULAR ) {

		MMatrix mReferTransM = mReferenceMatrix.transpose();
		MVector mVecSphere;

		for( ; !mItGeometry.isDone(); mItGeometry.next() ) {

			// Get the painted paramV value
			//double vertexWeight = weightValue( mDataBlock, multiIndex, mItGeometry.index() );

			// Result
			MVector mResult;

			for( int i = 0; i < numGeometries; ++i ) {
				mVecSphere = mPointArrays[ i ][ mItGeometry.index() ] - mItGeometry.position();		// Create delta shape
				mVecSphere *= mReferenceMatrix;														// Transform into Z-Axis alligned reference matrix space
				mVecSphere = mVecSphere.rotateBy( MVector::kZaxis , - i * 0.5 * M_PI );				// Rotate the delta vectors into the first spherical quadrant
				mResult += triClamp( fmod( paramU + rangeU + 1.0f - i, rangeU ) ) * mVecSphere;		// Linear interpolation
			}

			mResult = mResult.rotateBy( MVector::kZaxis , phi );			// Rotate solution arround Z-Axis with phi
			mResult *= mReferTransM;										// Transform back into Model Space
			mResult = paramV * mResult + mItGeometry.position();			// Apply linearly interpolated Delta Vctors
			mItGeometry.setPosition( mResult );								// Set the new output point
		}
	}


	// C-Spline (Circular B-Spline) interpolation, B_Spline interpolation in cylindrical coordinates
	else if( interp == C_SPLINE ) {

		computeCSpline(
			mReferenceMatrix,
			paramU,
			paramV,
			mItGeometry,
			& mPointArrays[0],
			true );

	}

	// C-Spline (Circular B-Spline) interpolation, B_Spline interpolation in (quasi-) cylindrical coordinates
	else if( interp == C_TEMP ) {

		computeCSpline(
			mReferenceMatrix,
			paramU,
			paramV,
			mItGeometry,
			& mPointArrays[0],
			false );


		/*
		MMatrix mReferTransM = mReferenceMatrix.transpose();

		unsigned int intParamU = std::min(( int )( paramU ), 3 );	// Must not become 4
		unsigned int intParamU_1 = ( intParamU + 1 ) % 4;

		double t1 = paramU - intParamU;
		double t2 = t1 * t1;
		double t3 = t2 * t1;

		MVector mVecSphere;
		MPointArray mDifference( 3 ), mD( 3 );
		MVectorArray mDeltas( 4 );


		for( ; !mItGeometry.isDone(); mItGeometry.next() ) {

			// Get the painted paramV value
			//double vertexWeight = weightValue( mDataBlock, multiIndex, mItGeometry.index() );

			// Result
			MVector mResult;

			// Compute delta shape between deformed orignal shape and each target shape
			// Transform each delta shape into refernce matrix space
			// Rotat deltas from second to last quadrant into the first quadrant, so that all deltas point into similar
			for( int i = 0; i < 4; ++i ) {
				mDeltas[i] = mPointArrays[ i ][ mItGeometry.index() ] - mItGeometry.position();		// Create delta shape
				mDeltas[i] *= mReferenceMatrix;														// Transforme into refernece matrix space
				if( i > 0 )
					mDeltas[i] = mDeltas[i].rotateBy( MVector::kZaxis , - i * 0.5 * M_PI );			// Rotate the delta vectors into the first spherical quadrant
			}

			for( int i = 0; i < 4; ++i ) {
				mResult = 3 * ( mDeltas[ ( i + 1 + 4 ) % 4 ] - mDeltas[ ( i - 1 + 4 ) % 4 ] );
				mDifference[0][i] = mResult.x;
				mDifference[1][i] = mResult.y;
				mDifference[2][i] = mResult.z;
			}

			// Compute cofficiencies
			for( int i = 0; i < 3; ++i ) {
				mD[i] = mDifference[i] * matA_inverted;
			}

			// Compute Coefficiencies
			MVector a = mDeltas[ intParamU ];
			MVector a_1 = mDeltas[ intParamU_1 ];
			MVector b( mD[0][ intParamU ], mD[1][ intParamU ], mD[2][ intParamU ] );
			MVector b_1( mD[0][ intParamU_1 ], mD[1][ intParamU_1 ], mD[2][ intParamU_1 ] );
			MVector c = 3 * ( a_1 - a ) - 2 * b - b_1;
			MVector d = 2 * ( a - a_1 ) +     b + b_1;

			mResult = a + b * t1 + c * t2 + d * t3;

			mResult = mResult.rotateBy( MVector::kZaxis, phi );
			mResult *= mReferTransM;

			mResult = paramV * mResult + mItGeometry.position();

			// Set the new output point
			mItGeometry.setPosition( mResult );
		}*/
	}

	// Maya NURBs API Spline - very slow!
	else if( interp == M_NURBS ) {

		// Result
		MPoint mPoint;
		MPointArray mEditPoints;
		mEditPoints.setLength(numGeometries + 1);

		MFnNurbsCurve mFnNurbsCurve;
		MFnNurbsCurveData mFnNurbsCurveData;
		MObject mNurbsCurveData = mFnNurbsCurveData.create(&mStatus);

		for( ; !mItGeometry.isDone(); mItGeometry.next()) {

			// B-Spline ( NURBs API ) deformation, First and Last Point must be the same for a Periodic Curve
			mEditPoints[0] = mEditPoints[ numGeometries ] = mPointArrays[0][ mItGeometry.index() ];
			for (int i = 1; i < numGeometries; ++i)
				mEditPoints[i] = mPointArrays[ i ][ mItGeometry.index() ];

			MObject mCurveGeom = mFnNurbsCurve.createWithEditPoints(
				mEditPoints, 3, MFnNurbsCurve::kPeriodic, false, false, true, mNurbsCurveData, &mStatus );

			mStatus = mFnNurbsCurve.getPointAtParam( paramU, mPoint );

			// Get the painted paramV value
			//double vertexWeight = weightValue( mDataBlock, multiIndex, mItGeometry.index() );

			mPoint = paramV * mPoint + invPaV * mItGeometry.position(); // * vertexWeigh
																		// Set the new output point
			mItGeometry.setPosition( mPoint );
		}
	}

	else {
		delete[] mPointArrays;
		return MS::kFailure;
	}

	delete[] mPointArrays;
	return MS::kSuccess;


}

/*
inline void computeCSpline(
	MMatrix & mReferenceMatrix,
	double paramU,
	double paramV,
	MItGeometry & mItGeometry,
	MPointArray * mPointArrays,
	bool toCartesian
	) {

	const int tmp_start = 106;
	const int tmp_count = 4;
	MVector mc_0[tmp_count];
	MVector mc_1[tmp_count];
	MVector mc_2[tmp_count];
	MVector mc_3[tmp_count];
	MVector mc_4[tmp_count];
	MVector mc_5[tmp_count];
	MVector md_0[tmp_count];
	MVector md_1[tmp_count];
	MVector md_2[tmp_count];
	MVector md_3[tmp_count];
	MVector md_4[tmp_count];
	MVector md_5[tmp_count];
	MPoint  df_0[tmp_count];
	MPoint  df_1[tmp_count];
	MPoint  df_2[tmp_count];


	unsigned int intParamU = std::min(( int )( paramU ), 3 );	// Must not become 4
	unsigned int intParamU_1 = (( intParamU + 1 ) % 4 ) + 1;
	intParamU++;
	intParamU_1++;

	double t1 = paramU - intParamU;
	double t2 = t1 * t1;
	double t3 = t2 * t1;

	MVector mVecSphere;
	MPointArray mDifference( 3 ), mD( 3 );
	MVectorArray mDeltas( 6 );
	MVectorArray mDeltasCart( 6 );

	for( ; !mItGeometry.isDone(); mItGeometry.next() ) {

		// Get the painted paramV value
		//double vertexWeight = weightValue( mDataBlock, multiIndex, mItGeometry.index() );

		// Result
		MVector mResult;

		// Test if cartesian2... and ...2cartesian work properly
		//mResult = MVector( mItGeometry.position());
		//cartesian2spherical( mResult );
		//spherical2cartesian( mResult );
		//cartesian2cylindrical( mResult );
		//cylindrical2cartesian( mResult );
		//mItGeometry.setPosition( mResult );
		//continue;



		// Compute delta shape between deformed orignal shape and each target shape
		// Transform each delta shape into refernce matrix space
		// Rotat deltas from second to last quadrant into the first quadrant, so that all deltas point into similar
		for( int i = 1; i < 5; ++i ) {
			mDeltasCart[i] = mPointArrays[i-1][ mItGeometry.index() ] - mItGeometry.position();		// Create delta shape
			mDeltas[i] = mDeltasCart[i] * mReferenceMatrix;														// Transforme into refernece matrix space
			cartesian2cylindrical( mDeltas[i] );												// Convert to cylindrical coordinates

			//mDeltas[i].y = fmod( mDeltas[i].y + M_PI, 2 * M_PI ) - M_PI;
			mDeltas[i].y += 2 * M_PI;
			// Todo(pp): actually convert to cylindrical, and then add ( 2 - i * 0.5 ) * M_PI to cylindrical phi
			//mDeltas[i].y -= i * 0.5 * M_PI;			// Rotate the delta vectors into the first spherical quadrant
		}


		//for( int i = 2; i < 4; ++i ) {
		//	auto mDelta_A = mDeltas[ ( i + 1 + 4 ) % 4 ];
		//	auto mDelta_B = mDeltas[ ( i - 1 + 4 ) % 4 ];
		//	//mDelta_A.y -= i * 0.5 * M_PI;
		//	//mDelta_B.y -= i * 0.5 * M_PI;
		//	mResult = 3 * ( mDelta_A - mDelta_B );

		//	mDifference[0][i] = mResult.x;
		//	mDifference[1][i] = mResult.y;
		//	mDifference[2][i] = mResult.z;
		//}

		mDeltas[5] = mDeltas[1];
		if( mDeltas[1].y >  2 * M_PI )
			mDeltas[1].y -= 2 * M_PI;
		else
			mDeltas[5].y += 2 * M_PI;

		{
			auto mDelta_A = mDeltas[ 2 ];
			auto mDelta_B = mDeltas[ 4 ];	// -1
			//mDelta_A.y -= 2 * M_PI;
			mDelta_B.y -= 2 * M_PI;
			mResult = 3 * ( mDelta_A - mDelta_B );
			mDifference[0][0] = mResult.x;
			mDifference[1][0] = mResult.y;
			mDifference[2][0] = mResult.z;
		}

		{
			auto mDelta_A = mDeltas[ 3 ];
			auto mDelta_B = mDeltas[ 1 ];
			//mDelta_A.y += 2 * M_PI;
			//mDelta_B.y -= 2 * M_PI;
			mResult = 3 * ( mDelta_A - mDelta_B );
			mDifference[0][1] = mResult.x;
			mDifference[1][1] = mResult.y;
			mDifference[2][1] = mResult.z;
		}

		{
			auto mDelta_A = mDeltas[ 4 ];
			auto mDelta_B = mDeltas[ 2 ];
			//mDelta_A.y -= 2 * M_PI;
			//mDelta_B.y -= 2 * M_PI;
			mResult = 3 * ( mDelta_A - mDelta_B );
			mDifference[0][2] = mResult.x;
			mDifference[1][2] = mResult.y;
			mDifference[2][2] = mResult.z;
		}

		{
			auto mDelta_A = mDeltas[ 5 ];
			auto mDelta_B = mDeltas[ 3 ];
			//mDelta_A.y -= 2 * M_PI;
			//mDelta_B.y += 2 * M_PI;
			mResult = 3 * ( mDelta_A - mDelta_B );
			mDifference[0][3] = mResult.x;
			mDifference[1][3] = mResult.y;
			mDifference[2][3] = mResult.z;
		}

		auto idx = mItGeometry.index();
		if( idx >= tmp_start && idx < tmp_start + tmp_count ) {
			auto ti = idx - tmp_start;
			mc_0[ti] = mDeltasCart[0];
			mc_1[ti] = mDeltasCart[1];
			mc_2[ti] = mDeltasCart[2]; 
			mc_3[ti] = mDeltasCart[3];
			mc_4[ti] = mDeltasCart[4];
			mc_5[ti] = mDeltasCart[5];
			md_0[ti] = mDeltas[0];
			md_1[ti] = mDeltas[1];
			md_2[ti] = mDeltas[2];
			md_3[ti] = mDeltas[3];
			md_4[ti] = mDeltas[4];
			md_5[ti] = mDeltas[5];
			df_0[ti] = mDifference[0];
			df_1[ti] = mDifference[1];
			df_2[ti] = mDifference[2];

		}

		// Compute cofficiencies
		for( int i = 0; i < 3; ++i ) {
			mD[i] = mDifference[i] * matA_inverted;
		}

		// Compute Coefficiencies
		MVector a = mDeltas[ intParamU ];
		MVector a_1 = mDeltas[ intParamU + 1 ];
		MVector b( mD[0][ intParamU ], mD[1][ intParamU ], mD[2][ intParamU ] );
		MVector b_1( mD[0][ intParamU_1 ], mD[1][ intParamU_1 ], mD[2][ intParamU_1 ] );
		MVector c = 3 * ( a_1 - a ) - 2 * b - b_1;
		MVector d = 2 * ( a - a_1 ) +     b + b_1;

		mResult = a + b * t1 + c * t2 + d * t3;

		//mResult = mResult.rotateBy( MVector::kZaxis, phi );
		//mResult.y += phi;
		if( toCartesian )
			cylindrical2cartesian( mResult );

		mResult = mReferenceMatrix * mResult;

		mResult = paramV * mResult + mItGeometry.position();

		// Set the new output point
		mItGeometry.setPosition( mResult );

	}
}
*/