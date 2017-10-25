
#include "CubicBlendNode.h"
#include "SphericalCoordinates.h"
#include "api_macros.h"


#include <maya/MGlobal.h>
#include <maya/MObject.h>
#include <maya/MStatus.h>
#include <maya/MFnPlugin.h>



MStatus initializePlugin( MObject mObject ) {
	MGlobal::displayInfo( "Initializing Bachelor PlugIn !" );
	MStatus mStatus;
	MFnPlugin mFnPlugin( mObject, "Peter Particle", "1.0", "Any" );

	// Specify that we are making a deformer node
	mStatus = mFnPlugin.registerNode( "cubicBlendNode", CubicBlendNode::id, CubicBlendNode::creator, CubicBlendNode::initialize, MPxNode::kDeformerNode );
	CHECK_MSTATUS_AND_RETURN_IT( mStatus );

	// Specify that we are making a DG Node
	mStatus = mFnPlugin.registerNode( "sphericalCoordinates", SphericalCoordinates::id, SphericalCoordinates::creator, SphericalCoordinates::initialize );
	CHECK_MSTATUS_AND_RETURN_IT( mStatus );

	return mStatus;
}


MStatus uninitializePlugin( MObject mObject ) {
	MStatus		mStatus;
	MFnPlugin	mFnPlugin( mObject );

	mStatus = mFnPlugin.deregisterNode( CubicBlendNode::id );
	CHECK_MSTATUS_AND_RETURN_IT( mStatus );

	mStatus = mFnPlugin.deregisterNode( SphericalCoordinates::id );
	CHECK_MSTATUS_AND_RETURN_IT( mStatus );

	return mStatus;
}