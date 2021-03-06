//#ifndef MOUSEPICKER_H
//#define MOUSEPICKER_H
//
//#include "btBulletDynamicsCommon.h"
//#include <iostream>
//using namespace std;
//
//class Mousepicker
//{
//	public:
//		Mousepicker(btDynamicsWorld* btWorld);
//		~Mousepicker();
//
//		void		attach( btRigidBody* pickBody, const btVector3& attachPosition, const btVector3& rayFrom, const btVector3& rayTo );
//		void		detach();
//		void		moveTo( const btVector3& origin, const btVector3& newdirection );
//		void		moveFrom( const btVector3& origin );
//
//		bool*				pickedBool;
//		bool				active;
//
//                btRigidBody* getPickedBody() { return pickedBody; }
//
//	private:
//		btDynamicsWorld*		btDynWorld;
//
//		btVector3			direction;
//		btRigidBody*			pickedBody;
//		btPoint2PointConstraint*	constraint;
//		float				oldPickingDist;
//};
//
//#endif
