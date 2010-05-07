#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=
AS=as

# Macros
CND_PLATFORM=GNU-Linux-x86
CND_CONF=Release
CND_DISTDIR=dist

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=build/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/crittergod/neural/NInput.o \
	${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btOverlappingPairCache.o \
	${OBJECTDIR}/crittergod/widget3d/WidgetPlane.o \
	${OBJECTDIR}/bullet/OpenGL/RenderTexture.o \
	${OBJECTDIR}/crittergod/video/GLWindow.o \
	${OBJECTDIR}/bullet/BulletCollision/Gimpact/gim_box_set.o \
	${OBJECTDIR}/crittergod/neural/BrainLink.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btGhostObject.o \
	${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.o \
	${OBJECTDIR}/crittergod/math/vector2i.o \
	${OBJECTDIR}/bullet/BulletDynamics/Dynamics/btContinuousDynamicsWorld.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btTriangleMesh.o \
	${OBJECTDIR}/bullet/BulletCollision/Gimpact/btGImpactShape.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btManifoldResult.o \
	${OBJECTDIR}/bullet/BulletSoftBody/btSoftBodyHelpers.o \
	${OBJECTDIR}/crittergod/bio/ServoHinge.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.o \
	${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btHinge2Constraint.o \
	${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btBroadphaseProxy.o \
	${OBJECTDIR}/crittergod/widget2d/panel.o \
	${OBJECTDIR}/crittergod/widget3d/TextRect.o \
	${OBJECTDIR}/bullet/BulletSoftBody/btSoftBodyConcaveCollisionAlgorithm.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btConvex2dShape.o \
	${OBJECTDIR}/crittergod/video/mousepicker.o \
	${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btPersistentManifold.o \
	${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btUniversalConstraint.o \
	${OBJECTDIR}/bullet/BulletDynamics/Dynamics/btRigidBody.o \
	${OBJECTDIR}/crittergod/widget2d/widget.o \
	${OBJECTDIR}/crittergod/bio/SixDoFMotor.o \
	${OBJECTDIR}/crittergod/bio/SineSound.o \
	${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btSimpleBroadphase.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btCompoundShape.o \
	${OBJECTDIR}/bullet/OpenGL/GLDebugDrawer.o \
	${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.o \
	${OBJECTDIR}/bullet/LinearMath/btSerializer.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btTriangleBuffer.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btTriangleMeshShape.o \
	${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btMultiSapBroadphase.o \
	${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btGjkEpa2.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btPolyhedralConvexShape.o \
	${OBJECTDIR}/bullet/BulletCollision/Gimpact/gim_contact.o \
	${OBJECTDIR}/bullet/BulletSoftBody/btSoftSoftCollisionAlgorithm.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.o \
	${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.o \
	${OBJECTDIR}/bullet/BulletCollision/Gimpact/btTriangleShapeEx.o \
	${OBJECTDIR}/crittergod/RunTests.o \
	${OBJECTDIR}/bullet/BulletCollision/Gimpact/gim_tri_collision.o \
	${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btDbvt.o \
	${OBJECTDIR}/crittergod/video/gldebugdrawer.o \
	${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btSliderConstraint.o \
	${OBJECTDIR}/crittergod/neural/NOutput.o \
	${OBJECTDIR}/crittergod/video/raycast.o \
	${OBJECTDIR}/crittergod/objects/RetinaPanel.o \
	${OBJECTDIR}/crittergod/graph/Graph.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btBoxBoxDetector.o \
	${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btRaycastCallback.o \
	${OBJECTDIR}/bullet/BulletDynamics/Dynamics/Bullet-C-API.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btCylinderShape.o \
	${OBJECTDIR}/crittergod/widget3d/Rect.o \
	${OBJECTDIR}/crittergod/widget2d/button.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btUnionFind.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btMultiSphereShape.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.o \
	${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.o \
	${OBJECTDIR}/crittergod/math/vector3f.o \
	${OBJECTDIR}/bullet/OpenGL/GL_DialogWindow.o \
	${OBJECTDIR}/bullet/BulletCollision/Gimpact/gim_memory.o \
	${OBJECTDIR}/crittergod/video/AbstractSpace.o \
	${OBJECTDIR}/crittergod/space/SnakeBody.o \
	${OBJECTDIR}/crittergod/math/Math.o \
	${OBJECTDIR}/crittergod/bio/NColor.o \
	${OBJECTDIR}/bullet/LinearMath/btConvexHull.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btUniformScalingShape.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.o \
	${OBJECTDIR}/crittergod/widget3d/Panel.o \
	${OBJECTDIR}/crittergod/space/RigidBody.o \
	${OBJECTDIR}/crittergod/video/displaylists.o \
	${OBJECTDIR}/crittergod/widget2d/textprinter.o \
	${OBJECTDIR}/crittergod/bio/BloodBrainInterface.o \
	${OBJECTDIR}/crittergod/objects/BrainVis.o \
	${OBJECTDIR}/crittergod/widget3d/Slider.o \
	${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btHingeConstraint.o \
	${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.o \
	${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btQuantizedBvh.o \
	${OBJECTDIR}/bullet/BulletSoftBody/btSoftRigidCollisionAlgorithm.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btEmptyShape.o \
	${OBJECTDIR}/crittergod/video/Spacetime.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btMinkowskiSumShape.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btCollisionWorld.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.o \
	${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.o \
	${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.o \
	${OBJECTDIR}/bullet/BulletSoftBody/btSoftRigidDynamicsWorld.o \
	${OBJECTDIR}/bullet/BulletCollision/Gimpact/btContactProcessing.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btBox2dShape.o \
	${OBJECTDIR}/bullet/OpenGL/GL_DialogDynamicsWorld.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btConeShape.o \
	${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btAxisSweep3.o \
	${OBJECTDIR}/crittergod/space/DefaultSpace.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/SphereTriangleDetector.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btCollisionShape.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.o \
	${OBJECTDIR}/bullet/OpenGL/GL_ShapeDrawer.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btConvexShape.o \
	${OBJECTDIR}/crittergod/widget2d/text.o \
	${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.o \
	${OBJECTDIR}/bullet/BulletCollision/Gimpact/btGImpactBvh.o \
	${OBJECTDIR}/bullet/OpenGL/GL_Simplex1to4.o \
	${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btConvexCast.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btSimulationIslandManager.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btShapeHull.o \
	${OBJECTDIR}/bullet/LinearMath/btGeometryUtil.o \
	${OBJECTDIR}/crittergod/audio/SoundSource.o \
	${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btTypedConstraint.o \
	${OBJECTDIR}/crittergod/objects/BrainPanel.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.o \
	${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btCollisionDispatcher.o \
	${OBJECTDIR}/crittergod/bio/NPosition.o \
	${OBJECTDIR}/crittergod/objects/FDBrainBody.o \
	${OBJECTDIR}/crittergod/RunSims.o \
	${OBJECTDIR}/bullet/LinearMath/btQuickprof.o \
	${OBJECTDIR}/crittergod/widget2d/container.o \
	${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btTetrahedronShape.o \
	${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.o \
	${OBJECTDIR}/crittergod/neural/Neuron.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btConvexHullShape.o \
	${OBJECTDIR}/bullet/BulletDynamics/Dynamics/btSimpleDynamicsWorld.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btCapsuleShape.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btConvexPointCloudShape.o \
	${OBJECTDIR}/bullet/OpenGL/Win32DemoApplication.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btOptimizedBvh.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btBoxShape.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.o \
	${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btTriangleCallback.o \
	${OBJECTDIR}/bullet/BulletDynamics/Vehicle/btWheelInfo.o \
	${OBJECTDIR}/crittergod/space/Humanoid.o \
	${OBJECTDIR}/crittergod/objects/PointerPanel.o \
	${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btContactConstraint.o \
	${OBJECTDIR}/crittergod/widget3d/Button.o \
	${OBJECTDIR}/bullet/BulletCollision/Gimpact/btGImpactQuantizedBvh.o \
	${OBJECTDIR}/bullet/LinearMath/btAlignedAllocator.o \
	${OBJECTDIR}/bullet/OpenGL/GlutStuff.o \
	${OBJECTDIR}/bullet/OpenGL/Win32AppMain.o \
	${OBJECTDIR}/crittergod/math/vector2f.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.o \
	${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btCollisionObject.o \
	${OBJECTDIR}/bullet/BulletDynamics/Character/btKinematicCharacterController.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.o \
	${OBJECTDIR}/bullet/BulletSoftBody/btSoftBody.o \
	${OBJECTDIR}/bullet/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.o \
	${OBJECTDIR}/crittergod/RunWidgets.o \
	${OBJECTDIR}/bullet/OpenGL/GLDebugFont.o \
	${OBJECTDIR}/bullet/BulletCollision/Gimpact/btGenericPoolAllocator.o \
	${OBJECTDIR}/crittergod/objects/NeuralSignalsPanel.o \
	${OBJECTDIR}/crittergod/space/SpiderBody.o \
	${OBJECTDIR}/crittergod/widget2d/slider.o \
	${OBJECTDIR}/bullet/BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.o \
	${OBJECTDIR}/crittergod/video/SpaceProcess.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btConcaveShape.o \
	${OBJECTDIR}/crittergod/bio/Retina.o \
	${OBJECTDIR}/crittergod/audio/Audio.o \
	${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btConeTwistConstraint.o \
	${OBJECTDIR}/crittergod/RunRobots.o \
	${OBJECTDIR}/crittergod/video/font/FontDemo1.o \
	${OBJECTDIR}/main.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btSphereShape.o \
	${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btDispatcher.o \
	${OBJECTDIR}/crittergod/space/BoxBody.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btStaticPlaneShape.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.o \
	${OBJECTDIR}/crittergod/widget3d/XYSlider.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.o \
	${OBJECTDIR}/bullet/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btInternalEdgeUtility.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.o \
	${OBJECTDIR}/bullet/BulletDynamics/Vehicle/btRaycastVehicle.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btStridingMeshInterface.o \
	${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btDbvtBroadphase.o \
	${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btConvexInternalShape.o \
	${OBJECTDIR}/crittergod/neural/Brain.o

# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	${MAKE}  -f nbproject/Makefile-Release.mk dist/Release/GNU-Linux-x86/crittergod5

dist/Release/GNU-Linux-x86/crittergod5: ${OBJECTFILES}
	${MKDIR} -p dist/Release/GNU-Linux-x86
	${LINK.cc} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/crittergod5 ${OBJECTFILES} ${LDLIBSOPTIONS} 

${OBJECTDIR}/crittergod/neural/NInput.o: nbproject/Makefile-${CND_CONF}.mk crittergod/neural/NInput.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/neural
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/neural/NInput.o crittergod/neural/NInput.cpp

${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btOverlappingPairCache.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/BroadphaseCollision/btOverlappingPairCache.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btOverlappingPairCache.o bullet/BulletCollision/BroadphaseCollision/btOverlappingPairCache.cpp

${OBJECTDIR}/crittergod/widget3d/WidgetPlane.o: nbproject/Makefile-${CND_CONF}.mk crittergod/widget3d/WidgetPlane.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/widget3d
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/widget3d/WidgetPlane.o crittergod/widget3d/WidgetPlane.cpp

${OBJECTDIR}/bullet/OpenGL/RenderTexture.o: nbproject/Makefile-${CND_CONF}.mk bullet/OpenGL/RenderTexture.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/OpenGL
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/OpenGL/RenderTexture.o bullet/OpenGL/RenderTexture.cpp

${OBJECTDIR}/crittergod/video/GLWindow.o: nbproject/Makefile-${CND_CONF}.mk crittergod/video/GLWindow.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/video
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/video/GLWindow.o crittergod/video/GLWindow.cpp

${OBJECTDIR}/bullet/BulletCollision/Gimpact/gim_box_set.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/Gimpact/gim_box_set.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/Gimpact
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/Gimpact/gim_box_set.o bullet/BulletCollision/Gimpact/gim_box_set.cpp

${OBJECTDIR}/crittergod/neural/BrainLink.o: nbproject/Makefile-${CND_CONF}.mk crittergod/neural/BrainLink.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/neural
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/neural/BrainLink.o crittergod/neural/BrainLink.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btGhostObject.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionDispatch/btGhostObject.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btGhostObject.o bullet/BulletCollision/CollisionDispatch/btGhostObject.cpp

${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.o bullet/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.cpp

${OBJECTDIR}/crittergod/math/vector2i.o: nbproject/Makefile-${CND_CONF}.mk crittergod/math/vector2i.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/math
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/math/vector2i.o crittergod/math/vector2i.cpp

${OBJECTDIR}/bullet/BulletDynamics/Dynamics/btContinuousDynamicsWorld.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletDynamics/Dynamics/btContinuousDynamicsWorld.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletDynamics/Dynamics
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletDynamics/Dynamics/btContinuousDynamicsWorld.o bullet/BulletDynamics/Dynamics/btContinuousDynamicsWorld.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btTriangleMesh.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btTriangleMesh.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btTriangleMesh.o bullet/BulletCollision/CollisionShapes/btTriangleMesh.cpp

${OBJECTDIR}/bullet/BulletCollision/Gimpact/btGImpactShape.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/Gimpact/btGImpactShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/Gimpact
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/Gimpact/btGImpactShape.o bullet/BulletCollision/Gimpact/btGImpactShape.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btManifoldResult.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionDispatch/btManifoldResult.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btManifoldResult.o bullet/BulletCollision/CollisionDispatch/btManifoldResult.cpp

${OBJECTDIR}/bullet/BulletSoftBody/btSoftBodyHelpers.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletSoftBody/btSoftBodyHelpers.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletSoftBody
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletSoftBody/btSoftBodyHelpers.o bullet/BulletSoftBody/btSoftBodyHelpers.cpp

${OBJECTDIR}/crittergod/bio/ServoHinge.o: nbproject/Makefile-${CND_CONF}.mk crittergod/bio/ServoHinge.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/bio
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/bio/ServoHinge.o crittergod/bio/ServoHinge.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.o bullet/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.cpp

${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btHinge2Constraint.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletDynamics/ConstraintSolver/btHinge2Constraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btHinge2Constraint.o bullet/BulletDynamics/ConstraintSolver/btHinge2Constraint.cpp

${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btBroadphaseProxy.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/BroadphaseCollision/btBroadphaseProxy.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btBroadphaseProxy.o bullet/BulletCollision/BroadphaseCollision/btBroadphaseProxy.cpp

${OBJECTDIR}/crittergod/widget2d/panel.o: nbproject/Makefile-${CND_CONF}.mk crittergod/widget2d/panel.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/widget2d
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/widget2d/panel.o crittergod/widget2d/panel.cpp

${OBJECTDIR}/crittergod/widget3d/TextRect.o: nbproject/Makefile-${CND_CONF}.mk crittergod/widget3d/TextRect.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/widget3d
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/widget3d/TextRect.o crittergod/widget3d/TextRect.cpp

${OBJECTDIR}/bullet/BulletSoftBody/btSoftBodyConcaveCollisionAlgorithm.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletSoftBody/btSoftBodyConcaveCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletSoftBody
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletSoftBody/btSoftBodyConcaveCollisionAlgorithm.o bullet/BulletSoftBody/btSoftBodyConcaveCollisionAlgorithm.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btConvex2dShape.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btConvex2dShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btConvex2dShape.o bullet/BulletCollision/CollisionShapes/btConvex2dShape.cpp

${OBJECTDIR}/crittergod/video/mousepicker.o: nbproject/Makefile-${CND_CONF}.mk crittergod/video/mousepicker.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/video
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/video/mousepicker.o crittergod/video/mousepicker.cpp

${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btPersistentManifold.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/NarrowPhaseCollision/btPersistentManifold.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btPersistentManifold.o bullet/BulletCollision/NarrowPhaseCollision/btPersistentManifold.cpp

${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btUniversalConstraint.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletDynamics/ConstraintSolver/btUniversalConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btUniversalConstraint.o bullet/BulletDynamics/ConstraintSolver/btUniversalConstraint.cpp

${OBJECTDIR}/bullet/BulletDynamics/Dynamics/btRigidBody.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletDynamics/Dynamics/btRigidBody.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletDynamics/Dynamics
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletDynamics/Dynamics/btRigidBody.o bullet/BulletDynamics/Dynamics/btRigidBody.cpp

${OBJECTDIR}/crittergod/widget2d/widget.o: nbproject/Makefile-${CND_CONF}.mk crittergod/widget2d/widget.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/widget2d
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/widget2d/widget.o crittergod/widget2d/widget.cpp

${OBJECTDIR}/crittergod/bio/SixDoFMotor.o: nbproject/Makefile-${CND_CONF}.mk crittergod/bio/SixDoFMotor.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/bio
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/bio/SixDoFMotor.o crittergod/bio/SixDoFMotor.cpp

${OBJECTDIR}/crittergod/bio/SineSound.o: nbproject/Makefile-${CND_CONF}.mk crittergod/bio/SineSound.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/bio
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/bio/SineSound.o crittergod/bio/SineSound.cpp

${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btSimpleBroadphase.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/BroadphaseCollision/btSimpleBroadphase.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btSimpleBroadphase.o bullet/BulletCollision/BroadphaseCollision/btSimpleBroadphase.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btCompoundShape.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btCompoundShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btCompoundShape.o bullet/BulletCollision/CollisionShapes/btCompoundShape.cpp

${OBJECTDIR}/bullet/OpenGL/GLDebugDrawer.o: nbproject/Makefile-${CND_CONF}.mk bullet/OpenGL/GLDebugDrawer.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/OpenGL
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/OpenGL/GLDebugDrawer.o bullet/OpenGL/GLDebugDrawer.cpp

${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.o bullet/BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.cpp

${OBJECTDIR}/bullet/LinearMath/btSerializer.o: nbproject/Makefile-${CND_CONF}.mk bullet/LinearMath/btSerializer.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/LinearMath
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/LinearMath/btSerializer.o bullet/LinearMath/btSerializer.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btTriangleBuffer.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btTriangleBuffer.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btTriangleBuffer.o bullet/BulletCollision/CollisionShapes/btTriangleBuffer.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btTriangleMeshShape.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btTriangleMeshShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btTriangleMeshShape.o bullet/BulletCollision/CollisionShapes/btTriangleMeshShape.cpp

${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btMultiSapBroadphase.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/BroadphaseCollision/btMultiSapBroadphase.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btMultiSapBroadphase.o bullet/BulletCollision/BroadphaseCollision/btMultiSapBroadphase.cpp

${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btGjkEpa2.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/NarrowPhaseCollision/btGjkEpa2.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btGjkEpa2.o bullet/BulletCollision/NarrowPhaseCollision/btGjkEpa2.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btPolyhedralConvexShape.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btPolyhedralConvexShape.o bullet/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp

${OBJECTDIR}/bullet/BulletCollision/Gimpact/gim_contact.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/Gimpact/gim_contact.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/Gimpact
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/Gimpact/gim_contact.o bullet/BulletCollision/Gimpact/gim_contact.cpp

${OBJECTDIR}/bullet/BulletSoftBody/btSoftSoftCollisionAlgorithm.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletSoftBody/btSoftSoftCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletSoftBody
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletSoftBody/btSoftSoftCollisionAlgorithm.o bullet/BulletSoftBody/btSoftSoftCollisionAlgorithm.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.o bullet/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.cpp

${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.o bullet/BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.cpp

${OBJECTDIR}/bullet/BulletCollision/Gimpact/btTriangleShapeEx.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/Gimpact/btTriangleShapeEx.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/Gimpact
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/Gimpact/btTriangleShapeEx.o bullet/BulletCollision/Gimpact/btTriangleShapeEx.cpp

${OBJECTDIR}/crittergod/RunTests.o: nbproject/Makefile-${CND_CONF}.mk crittergod/RunTests.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/RunTests.o crittergod/RunTests.cpp

${OBJECTDIR}/bullet/BulletCollision/Gimpact/gim_tri_collision.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/Gimpact/gim_tri_collision.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/Gimpact
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/Gimpact/gim_tri_collision.o bullet/BulletCollision/Gimpact/gim_tri_collision.cpp

${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btDbvt.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/BroadphaseCollision/btDbvt.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btDbvt.o bullet/BulletCollision/BroadphaseCollision/btDbvt.cpp

${OBJECTDIR}/crittergod/video/gldebugdrawer.o: nbproject/Makefile-${CND_CONF}.mk crittergod/video/gldebugdrawer.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/video
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/video/gldebugdrawer.o crittergod/video/gldebugdrawer.cpp

${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btSliderConstraint.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletDynamics/ConstraintSolver/btSliderConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btSliderConstraint.o bullet/BulletDynamics/ConstraintSolver/btSliderConstraint.cpp

${OBJECTDIR}/crittergod/neural/NOutput.o: nbproject/Makefile-${CND_CONF}.mk crittergod/neural/NOutput.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/neural
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/neural/NOutput.o crittergod/neural/NOutput.cpp

${OBJECTDIR}/crittergod/video/raycast.o: nbproject/Makefile-${CND_CONF}.mk crittergod/video/raycast.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/video
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/video/raycast.o crittergod/video/raycast.cpp

${OBJECTDIR}/crittergod/objects/RetinaPanel.o: nbproject/Makefile-${CND_CONF}.mk crittergod/objects/RetinaPanel.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/objects
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/objects/RetinaPanel.o crittergod/objects/RetinaPanel.cpp

${OBJECTDIR}/crittergod/graph/Graph.o: nbproject/Makefile-${CND_CONF}.mk crittergod/graph/Graph.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/graph
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/graph/Graph.o crittergod/graph/Graph.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btBoxBoxDetector.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionDispatch/btBoxBoxDetector.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btBoxBoxDetector.o bullet/BulletCollision/CollisionDispatch/btBoxBoxDetector.cpp

${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btRaycastCallback.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/NarrowPhaseCollision/btRaycastCallback.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btRaycastCallback.o bullet/BulletCollision/NarrowPhaseCollision/btRaycastCallback.cpp

${OBJECTDIR}/bullet/BulletDynamics/Dynamics/Bullet-C-API.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletDynamics/Dynamics/Bullet-C-API.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletDynamics/Dynamics
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletDynamics/Dynamics/Bullet-C-API.o bullet/BulletDynamics/Dynamics/Bullet-C-API.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btCylinderShape.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btCylinderShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btCylinderShape.o bullet/BulletCollision/CollisionShapes/btCylinderShape.cpp

${OBJECTDIR}/crittergod/widget3d/Rect.o: nbproject/Makefile-${CND_CONF}.mk crittergod/widget3d/Rect.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/widget3d
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/widget3d/Rect.o crittergod/widget3d/Rect.cpp

${OBJECTDIR}/crittergod/widget2d/button.o: nbproject/Makefile-${CND_CONF}.mk crittergod/widget2d/button.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/widget2d
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/widget2d/button.o crittergod/widget2d/button.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btUnionFind.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionDispatch/btUnionFind.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btUnionFind.o bullet/BulletCollision/CollisionDispatch/btUnionFind.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btMultiSphereShape.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btMultiSphereShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btMultiSphereShape.o bullet/BulletCollision/CollisionShapes/btMultiSphereShape.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.o bullet/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.cpp

${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.o bullet/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.cpp

${OBJECTDIR}/crittergod/math/vector3f.o: nbproject/Makefile-${CND_CONF}.mk crittergod/math/vector3f.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/math
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/math/vector3f.o crittergod/math/vector3f.cpp

${OBJECTDIR}/bullet/OpenGL/GL_DialogWindow.o: nbproject/Makefile-${CND_CONF}.mk bullet/OpenGL/GL_DialogWindow.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/OpenGL
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/OpenGL/GL_DialogWindow.o bullet/OpenGL/GL_DialogWindow.cpp

${OBJECTDIR}/bullet/BulletCollision/Gimpact/gim_memory.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/Gimpact/gim_memory.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/Gimpact
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/Gimpact/gim_memory.o bullet/BulletCollision/Gimpact/gim_memory.cpp

${OBJECTDIR}/crittergod/video/AbstractSpace.o: nbproject/Makefile-${CND_CONF}.mk crittergod/video/AbstractSpace.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/video
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/video/AbstractSpace.o crittergod/video/AbstractSpace.cpp

${OBJECTDIR}/crittergod/space/SnakeBody.o: nbproject/Makefile-${CND_CONF}.mk crittergod/space/SnakeBody.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/space
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/space/SnakeBody.o crittergod/space/SnakeBody.cpp

${OBJECTDIR}/crittergod/math/Math.o: nbproject/Makefile-${CND_CONF}.mk crittergod/math/Math.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/math
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/math/Math.o crittergod/math/Math.cpp

${OBJECTDIR}/crittergod/bio/NColor.o: nbproject/Makefile-${CND_CONF}.mk crittergod/bio/NColor.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/bio
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/bio/NColor.o crittergod/bio/NColor.cpp

${OBJECTDIR}/bullet/LinearMath/btConvexHull.o: nbproject/Makefile-${CND_CONF}.mk bullet/LinearMath/btConvexHull.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/LinearMath
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/LinearMath/btConvexHull.o bullet/LinearMath/btConvexHull.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.o bullet/BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.o bullet/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btUniformScalingShape.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btUniformScalingShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btUniformScalingShape.o bullet/BulletCollision/CollisionShapes/btUniformScalingShape.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.o bullet/BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.cpp

${OBJECTDIR}/crittergod/widget3d/Panel.o: nbproject/Makefile-${CND_CONF}.mk crittergod/widget3d/Panel.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/widget3d
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/widget3d/Panel.o crittergod/widget3d/Panel.cpp

${OBJECTDIR}/crittergod/space/RigidBody.o: nbproject/Makefile-${CND_CONF}.mk crittergod/space/RigidBody.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/space
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/space/RigidBody.o crittergod/space/RigidBody.cpp

${OBJECTDIR}/crittergod/video/displaylists.o: nbproject/Makefile-${CND_CONF}.mk crittergod/video/displaylists.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/video
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/video/displaylists.o crittergod/video/displaylists.cpp

${OBJECTDIR}/crittergod/widget2d/textprinter.o: nbproject/Makefile-${CND_CONF}.mk crittergod/widget2d/textprinter.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/widget2d
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/widget2d/textprinter.o crittergod/widget2d/textprinter.cpp

${OBJECTDIR}/crittergod/bio/BloodBrainInterface.o: nbproject/Makefile-${CND_CONF}.mk crittergod/bio/BloodBrainInterface.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/bio
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/bio/BloodBrainInterface.o crittergod/bio/BloodBrainInterface.cpp

${OBJECTDIR}/crittergod/objects/BrainVis.o: nbproject/Makefile-${CND_CONF}.mk crittergod/objects/BrainVis.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/objects
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/objects/BrainVis.o crittergod/objects/BrainVis.cpp

${OBJECTDIR}/crittergod/widget3d/Slider.o: nbproject/Makefile-${CND_CONF}.mk crittergod/widget3d/Slider.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/widget3d
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/widget3d/Slider.o crittergod/widget3d/Slider.cpp

${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btHingeConstraint.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletDynamics/ConstraintSolver/btHingeConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btHingeConstraint.o bullet/BulletDynamics/ConstraintSolver/btHingeConstraint.cpp

${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.o bullet/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.cpp

${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btQuantizedBvh.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/BroadphaseCollision/btQuantizedBvh.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btQuantizedBvh.o bullet/BulletCollision/BroadphaseCollision/btQuantizedBvh.cpp

${OBJECTDIR}/bullet/BulletSoftBody/btSoftRigidCollisionAlgorithm.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletSoftBody/btSoftRigidCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletSoftBody
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletSoftBody/btSoftRigidCollisionAlgorithm.o bullet/BulletSoftBody/btSoftRigidCollisionAlgorithm.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btEmptyShape.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btEmptyShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btEmptyShape.o bullet/BulletCollision/CollisionShapes/btEmptyShape.cpp

${OBJECTDIR}/crittergod/video/Spacetime.o: nbproject/Makefile-${CND_CONF}.mk crittergod/video/Spacetime.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/video
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/video/Spacetime.o crittergod/video/Spacetime.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btMinkowskiSumShape.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btMinkowskiSumShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btMinkowskiSumShape.o bullet/BulletCollision/CollisionShapes/btMinkowskiSumShape.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btCollisionWorld.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionDispatch/btCollisionWorld.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btCollisionWorld.o bullet/BulletCollision/CollisionDispatch/btCollisionWorld.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.o bullet/BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.cpp

${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.o bullet/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.cpp

${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.o bullet/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.cpp

${OBJECTDIR}/bullet/BulletSoftBody/btSoftRigidDynamicsWorld.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletSoftBody/btSoftRigidDynamicsWorld.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletSoftBody
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletSoftBody/btSoftRigidDynamicsWorld.o bullet/BulletSoftBody/btSoftRigidDynamicsWorld.cpp

${OBJECTDIR}/bullet/BulletCollision/Gimpact/btContactProcessing.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/Gimpact/btContactProcessing.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/Gimpact
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/Gimpact/btContactProcessing.o bullet/BulletCollision/Gimpact/btContactProcessing.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btBox2dShape.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btBox2dShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btBox2dShape.o bullet/BulletCollision/CollisionShapes/btBox2dShape.cpp

${OBJECTDIR}/bullet/OpenGL/GL_DialogDynamicsWorld.o: nbproject/Makefile-${CND_CONF}.mk bullet/OpenGL/GL_DialogDynamicsWorld.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/OpenGL
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/OpenGL/GL_DialogDynamicsWorld.o bullet/OpenGL/GL_DialogDynamicsWorld.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btConeShape.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btConeShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btConeShape.o bullet/BulletCollision/CollisionShapes/btConeShape.cpp

${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btAxisSweep3.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/BroadphaseCollision/btAxisSweep3.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btAxisSweep3.o bullet/BulletCollision/BroadphaseCollision/btAxisSweep3.cpp

${OBJECTDIR}/crittergod/space/DefaultSpace.o: nbproject/Makefile-${CND_CONF}.mk crittergod/space/DefaultSpace.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/space
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/space/DefaultSpace.o crittergod/space/DefaultSpace.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/SphereTriangleDetector.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionDispatch/SphereTriangleDetector.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/SphereTriangleDetector.o bullet/BulletCollision/CollisionDispatch/SphereTriangleDetector.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btCollisionShape.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btCollisionShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btCollisionShape.o bullet/BulletCollision/CollisionShapes/btCollisionShape.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.o bullet/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.cpp

${OBJECTDIR}/bullet/OpenGL/GL_ShapeDrawer.o: nbproject/Makefile-${CND_CONF}.mk bullet/OpenGL/GL_ShapeDrawer.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/OpenGL
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/OpenGL/GL_ShapeDrawer.o bullet/OpenGL/GL_ShapeDrawer.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btConvexShape.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btConvexShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btConvexShape.o bullet/BulletCollision/CollisionShapes/btConvexShape.cpp

${OBJECTDIR}/crittergod/widget2d/text.o: nbproject/Makefile-${CND_CONF}.mk crittergod/widget2d/text.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/widget2d
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/widget2d/text.o crittergod/widget2d/text.cpp

${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.o bullet/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.cpp

${OBJECTDIR}/bullet/BulletCollision/Gimpact/btGImpactBvh.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/Gimpact/btGImpactBvh.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/Gimpact
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/Gimpact/btGImpactBvh.o bullet/BulletCollision/Gimpact/btGImpactBvh.cpp

${OBJECTDIR}/bullet/OpenGL/GL_Simplex1to4.o: nbproject/Makefile-${CND_CONF}.mk bullet/OpenGL/GL_Simplex1to4.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/OpenGL
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/OpenGL/GL_Simplex1to4.o bullet/OpenGL/GL_Simplex1to4.cpp

${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btConvexCast.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/NarrowPhaseCollision/btConvexCast.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btConvexCast.o bullet/BulletCollision/NarrowPhaseCollision/btConvexCast.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btSimulationIslandManager.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionDispatch/btSimulationIslandManager.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btSimulationIslandManager.o bullet/BulletCollision/CollisionDispatch/btSimulationIslandManager.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btShapeHull.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btShapeHull.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btShapeHull.o bullet/BulletCollision/CollisionShapes/btShapeHull.cpp

${OBJECTDIR}/bullet/LinearMath/btGeometryUtil.o: nbproject/Makefile-${CND_CONF}.mk bullet/LinearMath/btGeometryUtil.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/LinearMath
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/LinearMath/btGeometryUtil.o bullet/LinearMath/btGeometryUtil.cpp

${OBJECTDIR}/crittergod/audio/SoundSource.o: nbproject/Makefile-${CND_CONF}.mk crittergod/audio/SoundSource.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/audio
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/audio/SoundSource.o crittergod/audio/SoundSource.cpp

${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btTypedConstraint.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletDynamics/ConstraintSolver/btTypedConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btTypedConstraint.o bullet/BulletDynamics/ConstraintSolver/btTypedConstraint.cpp

${OBJECTDIR}/crittergod/objects/BrainPanel.o: nbproject/Makefile-${CND_CONF}.mk crittergod/objects/BrainPanel.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/objects
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/objects/BrainPanel.o crittergod/objects/BrainPanel.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.o bullet/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.cpp

${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.o bullet/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btCollisionDispatcher.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionDispatch/btCollisionDispatcher.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btCollisionDispatcher.o bullet/BulletCollision/CollisionDispatch/btCollisionDispatcher.cpp

${OBJECTDIR}/crittergod/bio/NPosition.o: nbproject/Makefile-${CND_CONF}.mk crittergod/bio/NPosition.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/bio
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/bio/NPosition.o crittergod/bio/NPosition.cpp

${OBJECTDIR}/crittergod/objects/FDBrainBody.o: nbproject/Makefile-${CND_CONF}.mk crittergod/objects/FDBrainBody.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/objects
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/objects/FDBrainBody.o crittergod/objects/FDBrainBody.cpp

${OBJECTDIR}/crittergod/RunSims.o: nbproject/Makefile-${CND_CONF}.mk crittergod/RunSims.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/RunSims.o crittergod/RunSims.cpp

${OBJECTDIR}/bullet/LinearMath/btQuickprof.o: nbproject/Makefile-${CND_CONF}.mk bullet/LinearMath/btQuickprof.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/LinearMath
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/LinearMath/btQuickprof.o bullet/LinearMath/btQuickprof.cpp

${OBJECTDIR}/crittergod/widget2d/container.o: nbproject/Makefile-${CND_CONF}.mk crittergod/widget2d/container.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/widget2d
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/widget2d/container.o crittergod/widget2d/container.cpp

${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.o bullet/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btTetrahedronShape.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btTetrahedronShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btTetrahedronShape.o bullet/BulletCollision/CollisionShapes/btTetrahedronShape.cpp

${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.o bullet/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.cpp

${OBJECTDIR}/crittergod/neural/Neuron.o: nbproject/Makefile-${CND_CONF}.mk crittergod/neural/Neuron.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/neural
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/neural/Neuron.o crittergod/neural/Neuron.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btConvexHullShape.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btConvexHullShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btConvexHullShape.o bullet/BulletCollision/CollisionShapes/btConvexHullShape.cpp

${OBJECTDIR}/bullet/BulletDynamics/Dynamics/btSimpleDynamicsWorld.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletDynamics/Dynamics/btSimpleDynamicsWorld.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletDynamics/Dynamics
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletDynamics/Dynamics/btSimpleDynamicsWorld.o bullet/BulletDynamics/Dynamics/btSimpleDynamicsWorld.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btCapsuleShape.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btCapsuleShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btCapsuleShape.o bullet/BulletCollision/CollisionShapes/btCapsuleShape.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.o bullet/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btConvexPointCloudShape.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btConvexPointCloudShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btConvexPointCloudShape.o bullet/BulletCollision/CollisionShapes/btConvexPointCloudShape.cpp

${OBJECTDIR}/bullet/OpenGL/Win32DemoApplication.o: nbproject/Makefile-${CND_CONF}.mk bullet/OpenGL/Win32DemoApplication.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/OpenGL
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/OpenGL/Win32DemoApplication.o bullet/OpenGL/Win32DemoApplication.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btOptimizedBvh.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btOptimizedBvh.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btOptimizedBvh.o bullet/BulletCollision/CollisionShapes/btOptimizedBvh.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.o bullet/BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btBoxShape.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btBoxShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btBoxShape.o bullet/BulletCollision/CollisionShapes/btBoxShape.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.o bullet/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.cpp

${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.o bullet/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btTriangleCallback.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btTriangleCallback.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btTriangleCallback.o bullet/BulletCollision/CollisionShapes/btTriangleCallback.cpp

${OBJECTDIR}/bullet/BulletDynamics/Vehicle/btWheelInfo.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletDynamics/Vehicle/btWheelInfo.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletDynamics/Vehicle
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletDynamics/Vehicle/btWheelInfo.o bullet/BulletDynamics/Vehicle/btWheelInfo.cpp

${OBJECTDIR}/crittergod/space/Humanoid.o: nbproject/Makefile-${CND_CONF}.mk crittergod/space/Humanoid.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/space
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/space/Humanoid.o crittergod/space/Humanoid.cpp

${OBJECTDIR}/crittergod/objects/PointerPanel.o: nbproject/Makefile-${CND_CONF}.mk crittergod/objects/PointerPanel.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/objects
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/objects/PointerPanel.o crittergod/objects/PointerPanel.cpp

${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btContactConstraint.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletDynamics/ConstraintSolver/btContactConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btContactConstraint.o bullet/BulletDynamics/ConstraintSolver/btContactConstraint.cpp

${OBJECTDIR}/crittergod/widget3d/Button.o: nbproject/Makefile-${CND_CONF}.mk crittergod/widget3d/Button.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/widget3d
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/widget3d/Button.o crittergod/widget3d/Button.cpp

${OBJECTDIR}/bullet/BulletCollision/Gimpact/btGImpactQuantizedBvh.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/Gimpact/btGImpactQuantizedBvh.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/Gimpact
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/Gimpact/btGImpactQuantizedBvh.o bullet/BulletCollision/Gimpact/btGImpactQuantizedBvh.cpp

${OBJECTDIR}/bullet/LinearMath/btAlignedAllocator.o: nbproject/Makefile-${CND_CONF}.mk bullet/LinearMath/btAlignedAllocator.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/LinearMath
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/LinearMath/btAlignedAllocator.o bullet/LinearMath/btAlignedAllocator.cpp

${OBJECTDIR}/bullet/OpenGL/GlutStuff.o: nbproject/Makefile-${CND_CONF}.mk bullet/OpenGL/GlutStuff.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/OpenGL
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/OpenGL/GlutStuff.o bullet/OpenGL/GlutStuff.cpp

${OBJECTDIR}/bullet/OpenGL/Win32AppMain.o: nbproject/Makefile-${CND_CONF}.mk bullet/OpenGL/Win32AppMain.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/OpenGL
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/OpenGL/Win32AppMain.o bullet/OpenGL/Win32AppMain.cpp

${OBJECTDIR}/crittergod/math/vector2f.o: nbproject/Makefile-${CND_CONF}.mk crittergod/math/vector2f.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/math
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/math/vector2f.o crittergod/math/vector2f.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.o bullet/BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.o bullet/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.cpp

${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.o bullet/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btCollisionObject.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionDispatch/btCollisionObject.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btCollisionObject.o bullet/BulletCollision/CollisionDispatch/btCollisionObject.cpp

${OBJECTDIR}/bullet/BulletDynamics/Character/btKinematicCharacterController.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletDynamics/Character/btKinematicCharacterController.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletDynamics/Character
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletDynamics/Character/btKinematicCharacterController.o bullet/BulletDynamics/Character/btKinematicCharacterController.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.o bullet/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.cpp

${OBJECTDIR}/bullet/BulletSoftBody/btSoftBody.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletSoftBody/btSoftBody.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletSoftBody
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletSoftBody/btSoftBody.o bullet/BulletSoftBody/btSoftBody.cpp

${OBJECTDIR}/bullet/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/Gimpact
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.o bullet/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.o bullet/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.o bullet/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.cpp

${OBJECTDIR}/crittergod/RunWidgets.o: nbproject/Makefile-${CND_CONF}.mk crittergod/RunWidgets.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/RunWidgets.o crittergod/RunWidgets.cpp

${OBJECTDIR}/bullet/OpenGL/GLDebugFont.o: nbproject/Makefile-${CND_CONF}.mk bullet/OpenGL/GLDebugFont.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/OpenGL
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/OpenGL/GLDebugFont.o bullet/OpenGL/GLDebugFont.cpp

${OBJECTDIR}/bullet/BulletCollision/Gimpact/btGenericPoolAllocator.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/Gimpact/btGenericPoolAllocator.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/Gimpact
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/Gimpact/btGenericPoolAllocator.o bullet/BulletCollision/Gimpact/btGenericPoolAllocator.cpp

${OBJECTDIR}/crittergod/objects/NeuralSignalsPanel.o: nbproject/Makefile-${CND_CONF}.mk crittergod/objects/NeuralSignalsPanel.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/objects
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/objects/NeuralSignalsPanel.o crittergod/objects/NeuralSignalsPanel.cpp

${OBJECTDIR}/crittergod/space/SpiderBody.o: nbproject/Makefile-${CND_CONF}.mk crittergod/space/SpiderBody.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/space
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/space/SpiderBody.o crittergod/space/SpiderBody.cpp

${OBJECTDIR}/crittergod/widget2d/slider.o: nbproject/Makefile-${CND_CONF}.mk crittergod/widget2d/slider.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/widget2d
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/widget2d/slider.o crittergod/widget2d/slider.cpp

${OBJECTDIR}/bullet/BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletSoftBody
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.o bullet/BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.cpp

${OBJECTDIR}/crittergod/video/SpaceProcess.o: nbproject/Makefile-${CND_CONF}.mk crittergod/video/SpaceProcess.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/video
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/video/SpaceProcess.o crittergod/video/SpaceProcess.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btConcaveShape.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btConcaveShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btConcaveShape.o bullet/BulletCollision/CollisionShapes/btConcaveShape.cpp

${OBJECTDIR}/crittergod/bio/Retina.o: nbproject/Makefile-${CND_CONF}.mk crittergod/bio/Retina.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/bio
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/bio/Retina.o crittergod/bio/Retina.cpp

${OBJECTDIR}/crittergod/audio/Audio.o: nbproject/Makefile-${CND_CONF}.mk crittergod/audio/Audio.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/audio
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/audio/Audio.o crittergod/audio/Audio.cpp

${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btConeTwistConstraint.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletDynamics/ConstraintSolver/btConeTwistConstraint.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletDynamics/ConstraintSolver/btConeTwistConstraint.o bullet/BulletDynamics/ConstraintSolver/btConeTwistConstraint.cpp

${OBJECTDIR}/crittergod/RunRobots.o: nbproject/Makefile-${CND_CONF}.mk crittergod/RunRobots.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/RunRobots.o crittergod/RunRobots.cpp

${OBJECTDIR}/crittergod/video/font/FontDemo1.o: nbproject/Makefile-${CND_CONF}.mk crittergod/video/font/FontDemo1.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/video/font
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/video/font/FontDemo1.o crittergod/video/font/FontDemo1.cpp

${OBJECTDIR}/main.o: nbproject/Makefile-${CND_CONF}.mk main.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/main.o main.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btSphereShape.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btSphereShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btSphereShape.o bullet/BulletCollision/CollisionShapes/btSphereShape.cpp

${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btDispatcher.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/BroadphaseCollision/btDispatcher.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btDispatcher.o bullet/BulletCollision/BroadphaseCollision/btDispatcher.cpp

${OBJECTDIR}/crittergod/space/BoxBody.o: nbproject/Makefile-${CND_CONF}.mk crittergod/space/BoxBody.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/space
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/space/BoxBody.o crittergod/space/BoxBody.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btStaticPlaneShape.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btStaticPlaneShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btStaticPlaneShape.o bullet/BulletCollision/CollisionShapes/btStaticPlaneShape.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.o bullet/BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.cpp

${OBJECTDIR}/crittergod/widget3d/XYSlider.o: nbproject/Makefile-${CND_CONF}.mk crittergod/widget3d/XYSlider.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/widget3d
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/widget3d/XYSlider.o crittergod/widget3d/XYSlider.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.o bullet/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.cpp

${OBJECTDIR}/bullet/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletDynamics/Dynamics
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.o bullet/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btInternalEdgeUtility.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionDispatch/btInternalEdgeUtility.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btInternalEdgeUtility.o bullet/BulletCollision/CollisionDispatch/btInternalEdgeUtility.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.o bullet/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.cpp

${OBJECTDIR}/bullet/BulletDynamics/Vehicle/btRaycastVehicle.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletDynamics/Vehicle/btRaycastVehicle.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletDynamics/Vehicle
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletDynamics/Vehicle/btRaycastVehicle.o bullet/BulletDynamics/Vehicle/btRaycastVehicle.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btStridingMeshInterface.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btStridingMeshInterface.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btStridingMeshInterface.o bullet/BulletCollision/CollisionShapes/btStridingMeshInterface.cpp

${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btDbvtBroadphase.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/BroadphaseCollision/btDbvtBroadphase.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/BroadphaseCollision/btDbvtBroadphase.o bullet/BulletCollision/BroadphaseCollision/btDbvtBroadphase.cpp

${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btConvexInternalShape.o: nbproject/Makefile-${CND_CONF}.mk bullet/BulletCollision/CollisionShapes/btConvexInternalShape.cpp 
	${MKDIR} -p ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/bullet/BulletCollision/CollisionShapes/btConvexInternalShape.o bullet/BulletCollision/CollisionShapes/btConvexInternalShape.cpp

${OBJECTDIR}/crittergod/neural/Brain.o: nbproject/Makefile-${CND_CONF}.mk crittergod/neural/Brain.cpp 
	${MKDIR} -p ${OBJECTDIR}/crittergod/neural
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/crittergod/neural/Brain.o crittergod/neural/Brain.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/Release
	${RM} dist/Release/GNU-Linux-x86/crittergod5

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
