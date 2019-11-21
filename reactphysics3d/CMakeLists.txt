cmake_minimum_required(VERSION 2.8.3)
project(reactphysics3d)

add_compile_options(-std=c++11)
set(ROS_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include/reactphysics3d/src
  LIBRARIES reactphysics3d
)

##########################
## Build reactphysics3d ##
##########################

include_directories(
  include/reactphysics3d/src
  ${catkin_INCLUDE_DIRS}
)

set (REACTPHYSICS3D_HEADERS
    "include/reactphysics3d/src/configuration.h"
    "include/reactphysics3d/src/decimal.h"
    "include/reactphysics3d/src/reactphysics3d.h"
    "include/reactphysics3d/src/body/Body.h"
    "include/reactphysics3d/src/body/CollisionBody.h"
    "include/reactphysics3d/src/body/RigidBody.h"
    "include/reactphysics3d/src/collision/ContactPointInfo.h"
    "include/reactphysics3d/src/collision/ContactManifoldInfo.h"
    "include/reactphysics3d/src/collision/broadphase/BroadPhaseAlgorithm.h"
    "include/reactphysics3d/src/collision/broadphase/DynamicAABBTree.h"
    "include/reactphysics3d/src/collision/narrowphase/CollisionDispatch.h"
    "include/reactphysics3d/src/collision/narrowphase/DefaultCollisionDispatch.h"
    "include/reactphysics3d/src/collision/narrowphase/GJK/VoronoiSimplex.h"
    "include/reactphysics3d/src/collision/narrowphase/GJK/GJKAlgorithm.h"
    "include/reactphysics3d/src/collision/narrowphase/SAT/SATAlgorithm.h"
    "include/reactphysics3d/src/collision/narrowphase/NarrowPhaseAlgorithm.h"
    "include/reactphysics3d/src/collision/narrowphase/SphereVsSphereAlgorithm.h"
    "include/reactphysics3d/src/collision/narrowphase/CapsuleVsCapsuleAlgorithm.h"
    "include/reactphysics3d/src/collision/narrowphase/SphereVsCapsuleAlgorithm.h"
    "include/reactphysics3d/src/collision/narrowphase/SphereVsConvexPolyhedronAlgorithm.h"
    "include/reactphysics3d/src/collision/narrowphase/CapsuleVsConvexPolyhedronAlgorithm.h"
    "include/reactphysics3d/src/collision/narrowphase/ConvexPolyhedronVsConvexPolyhedronAlgorithm.h"
    "include/reactphysics3d/src/collision/shapes/AABB.h"
    "include/reactphysics3d/src/collision/shapes/ConvexShape.h"
    "include/reactphysics3d/src/collision/shapes/ConvexPolyhedronShape.h"
    "include/reactphysics3d/src/collision/shapes/ConcaveShape.h"
    "include/reactphysics3d/src/collision/shapes/BoxShape.h"
    "include/reactphysics3d/src/collision/shapes/CapsuleShape.h"
    "include/reactphysics3d/src/collision/shapes/CollisionShape.h"
    "include/reactphysics3d/src/collision/shapes/ConvexMeshShape.h"
    "include/reactphysics3d/src/collision/shapes/SphereShape.h"
    "include/reactphysics3d/src/collision/shapes/TriangleShape.h"
    "include/reactphysics3d/src/collision/shapes/ConcaveMeshShape.h"
    "include/reactphysics3d/src/collision/shapes/HeightFieldShape.h"
    "include/reactphysics3d/src/collision/RaycastInfo.h"
    "include/reactphysics3d/src/collision/ProxyShape.h"
    "include/reactphysics3d/src/collision/TriangleVertexArray.h"
    "include/reactphysics3d/src/collision/PolygonVertexArray.h"
    "include/reactphysics3d/src/collision/TriangleMesh.h"
    "include/reactphysics3d/src/collision/PolyhedronMesh.h"
    "include/reactphysics3d/src/collision/HalfEdgeStructure.h"
    "include/reactphysics3d/src/collision/CollisionDetection.h"
    "include/reactphysics3d/src/collision/NarrowPhaseInfo.h"
    "include/reactphysics3d/src/collision/ContactManifold.h"
    "include/reactphysics3d/src/collision/ContactManifoldSet.h"
    "include/reactphysics3d/src/collision/MiddlePhaseTriangleCallback.h"
    "include/reactphysics3d/src/constraint/BallAndSocketJoint.h"
    "include/reactphysics3d/src/constraint/ContactPoint.h"
    "include/reactphysics3d/src/constraint/FixedJoint.h"
    "include/reactphysics3d/src/constraint/HingeJoint.h"
    "include/reactphysics3d/src/constraint/Joint.h"
    "include/reactphysics3d/src/constraint/SliderJoint.h"
    "include/reactphysics3d/src/engine/CollisionWorld.h"
    "include/reactphysics3d/src/engine/ConstraintSolver.h"
    "include/reactphysics3d/src/engine/ContactSolver.h"
    "include/reactphysics3d/src/engine/DynamicsWorld.h"
    "include/reactphysics3d/src/engine/EventListener.h"
    "include/reactphysics3d/src/engine/Island.h"
    "include/reactphysics3d/src/engine/Material.h"
    "include/reactphysics3d/src/engine/OverlappingPair.h"
    "include/reactphysics3d/src/engine/Timer.h"
    "include/reactphysics3d/src/engine/Timer.cpp"
    "include/reactphysics3d/src/collision/CollisionCallback.h"
    "include/reactphysics3d/src/collision/OverlapCallback.h"
    "include/reactphysics3d/src/mathematics/mathematics.h"
    "include/reactphysics3d/src/mathematics/mathematics_functions.h"
    "include/reactphysics3d/src/mathematics/Matrix2x2.h"
    "include/reactphysics3d/src/mathematics/Matrix3x3.h"
    "include/reactphysics3d/src/mathematics/Quaternion.h"
    "include/reactphysics3d/src/mathematics/Transform.h"
    "include/reactphysics3d/src/mathematics/Vector2.h"
    "include/reactphysics3d/src/mathematics/Vector3.h"
    "include/reactphysics3d/src/mathematics/Ray.h"
    "include/reactphysics3d/src/memory/MemoryAllocator.h"
    "include/reactphysics3d/src/memory/DefaultPoolAllocator.h"
    "include/reactphysics3d/src/memory/DefaultSingleFrameAllocator.h"
    "include/reactphysics3d/src/memory/DefaultAllocator.h"
    "include/reactphysics3d/src/memory/MemoryManager.h"
    "include/reactphysics3d/src/containers/Stack.h"
    "include/reactphysics3d/src/containers/LinkedList.h"
    "include/reactphysics3d/src/containers/List.h"
    "include/reactphysics3d/src/containers/Map.h"
    "include/reactphysics3d/src/containers/Set.h"
    "include/reactphysics3d/src/containers/Pair.h"
    "include/reactphysics3d/src/utils/Profiler.h"
    "include/reactphysics3d/src/utils/Logger.h"
)

set (REACTPHYSICS3D_SOURCES
    "include/reactphysics3d/src/body/Body.cpp"
    "include/reactphysics3d/src/body/CollisionBody.cpp"
    "include/reactphysics3d/src/body/RigidBody.cpp"
    "include/reactphysics3d/src/collision/ContactManifoldInfo.cpp"
    "include/reactphysics3d/src/collision/broadphase/BroadPhaseAlgorithm.cpp"
    "include/reactphysics3d/src/collision/broadphase/DynamicAABBTree.cpp"
    "include/reactphysics3d/src/collision/narrowphase/DefaultCollisionDispatch.cpp"
    "include/reactphysics3d/src/collision/narrowphase/GJK/VoronoiSimplex.cpp"
    "include/reactphysics3d/src/collision/narrowphase/GJK/GJKAlgorithm.cpp"
    "include/reactphysics3d/src/collision/narrowphase/SAT/SATAlgorithm.cpp"
    "include/reactphysics3d/src/collision/narrowphase/SphereVsSphereAlgorithm.cpp"
    "include/reactphysics3d/src/collision/narrowphase/CapsuleVsCapsuleAlgorithm.cpp"
    "include/reactphysics3d/src/collision/narrowphase/SphereVsCapsuleAlgorithm.cpp"
    "include/reactphysics3d/src/collision/narrowphase/SphereVsConvexPolyhedronAlgorithm.cpp"
    "include/reactphysics3d/src/collision/narrowphase/CapsuleVsConvexPolyhedronAlgorithm.cpp"
    "include/reactphysics3d/src/collision/narrowphase/ConvexPolyhedronVsConvexPolyhedronAlgorithm.cpp"
    "include/reactphysics3d/src/collision/shapes/AABB.cpp"
    "include/reactphysics3d/src/collision/shapes/ConvexShape.cpp"
    "include/reactphysics3d/src/collision/shapes/ConvexPolyhedronShape.cpp"
    "include/reactphysics3d/src/collision/shapes/ConcaveShape.cpp"
    "include/reactphysics3d/src/collision/shapes/BoxShape.cpp"
    "include/reactphysics3d/src/collision/shapes/CapsuleShape.cpp"
    "include/reactphysics3d/src/collision/shapes/CollisionShape.cpp"
    "include/reactphysics3d/src/collision/shapes/ConvexMeshShape.cpp"
    "include/reactphysics3d/src/collision/shapes/SphereShape.cpp"
    "include/reactphysics3d/src/collision/shapes/TriangleShape.cpp"
    "include/reactphysics3d/src/collision/shapes/ConcaveMeshShape.cpp"
    "include/reactphysics3d/src/collision/shapes/HeightFieldShape.cpp"
    "include/reactphysics3d/src/collision/RaycastInfo.cpp"
    "include/reactphysics3d/src/collision/ProxyShape.cpp"
    "include/reactphysics3d/src/collision/TriangleVertexArray.cpp"
    "include/reactphysics3d/src/collision/PolygonVertexArray.cpp"
    "include/reactphysics3d/src/collision/TriangleMesh.cpp"
    "include/reactphysics3d/src/collision/PolyhedronMesh.cpp"
    "include/reactphysics3d/src/collision/HalfEdgeStructure.cpp"
    "include/reactphysics3d/src/collision/CollisionDetection.cpp"
    "include/reactphysics3d/src/collision/NarrowPhaseInfo.cpp"
    "include/reactphysics3d/src/collision/ContactManifold.cpp"
    "include/reactphysics3d/src/collision/ContactManifoldSet.cpp"
    "include/reactphysics3d/src/collision/MiddlePhaseTriangleCallback.cpp"
    "include/reactphysics3d/src/constraint/BallAndSocketJoint.cpp"
    "include/reactphysics3d/src/constraint/ContactPoint.cpp"
    "include/reactphysics3d/src/constraint/FixedJoint.cpp"
    "include/reactphysics3d/src/constraint/HingeJoint.cpp"
    "include/reactphysics3d/src/constraint/Joint.cpp"
    "include/reactphysics3d/src/constraint/SliderJoint.cpp"
    "include/reactphysics3d/src/engine/CollisionWorld.cpp"
    "include/reactphysics3d/src/engine/ConstraintSolver.cpp"
    "include/reactphysics3d/src/engine/ContactSolver.cpp"
    "include/reactphysics3d/src/engine/DynamicsWorld.cpp"
    "include/reactphysics3d/src/engine/Island.cpp"
    "include/reactphysics3d/src/engine/Material.cpp"
    "include/reactphysics3d/src/engine/OverlappingPair.cpp"
    "include/reactphysics3d/src/engine/Timer.cpp"
    "include/reactphysics3d/src/collision/CollisionCallback.cpp"
    "include/reactphysics3d/src/mathematics/mathematics_functions.cpp"
    "include/reactphysics3d/src/mathematics/Matrix2x2.cpp"
    "include/reactphysics3d/src/mathematics/Matrix3x3.cpp"
    "include/reactphysics3d/src/mathematics/Quaternion.cpp"
    "include/reactphysics3d/src/mathematics/Transform.cpp"
    "include/reactphysics3d/src/mathematics/Vector2.cpp"
    "include/reactphysics3d/src/mathematics/Vector3.cpp"
    "include/reactphysics3d/src/memory/DefaultPoolAllocator.cpp"
    "include/reactphysics3d/src/memory/DefaultSingleFrameAllocator.cpp"
    "include/reactphysics3d/src/memory/MemoryManager.cpp"
    "include/reactphysics3d/src/utils/Profiler.cpp"
    "include/reactphysics3d/src/utils/Logger.cpp"
)

add_library(reactphysics3d ${REACTPHYSICS3D_HEADERS} ${REACTPHYSICS3D_SOURCES})

target_include_directories(reactphysics3d PUBLIC
              $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/reactphysics3d/src>
              $<INSTALL_INTERFACE:include/reactphysics3d>
)
set_target_properties(reactphysics3d  PROPERTIES
          VERSION "0.7.1"
          SOVERSION "0.7"
)


add_dependencies(reactphysics3d ${catkin_EXPORTED_TARGETS})
target_link_libraries(reactphysics3d ${catkin_LIBRARIES})
