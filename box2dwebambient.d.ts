//
// box2dambient : TypeScript Declaration Source File for box2dweb
//
// Copyright 2012 Kon
// 
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
// 
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
// LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
// OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

module Box2D{
	export module Collision{
		interface IBroadPhase{
			CreateProxy(aabb:b2AABB, userData:any):any; 	 	
			DestroyProxy(proxy:any):void;
			GetFatAABB(proxy:any):b2AABB;
			GetProxyCount():number;
			GetUserData(proxy:any):any;
			MoveProxy(proxy:any, aabb:b2AABB, displacement:Box2D.Common.Math.b2Vec2):void;
			Query(callback:(subInput:b2RayCastInput, proxy:any)=>number, aabb:b2AABB):void;
			RayCast(callback:(subInput:b2RayCastInput, proxy:any)=>number, input:Box2D.Collision.b2RayCastInput):void;
			Rebalance(iterations:number):void;
			TestOverlap(proxyA:any, proxyB:any):bool;
			UpdatePairs(callback:(userDataA:any, userDataB:any)=>void):void;
			Validate():void;
		}
		export class b2AABB{
			lowerBound : Box2D.Common.Math.b2Vec2;
	 	 	upperBound : Box2D.Common.Math.b2Vec2;
			static Combine(aabb1:b2AABB, aabb2:b2AABB):b2AABB;
			Contains(aabb:b2AABB):bool;
			GetCenter():Box2D.Common.Math.b2Vec2;
			GetExtents():Box2D.Common.Math.b2Vec2;
			IsValid():bool;
			RayCast(output:Box2D.Collision.b2RayCastOutput, input:Box2D.Collision.b2RayCastInput):bool;
			TestOverlap(other:b2AABB):bool;
		}
		export class b2ContactID{
			features : Features;
	 	 	key : number;
	 	 	constructor();
			Copy():b2ContactID;
			Set(id:b2ContactID):void;
		}
		export class b2ContactPoint{
			friction : number;
	 	 	id : b2ContactID;
	 	 	normal : Box2D.Common.Math.b2Vec2;
	 	 	position : Box2D.Common.Math.b2Vec2;
	 	 	restitution : number;
	 	 	separation : number;
	 	 	shape1 : Box2D.Collision.Shapes.b2Shape;
	 	 	shape2 : Box2D.Collision.Shapes.b2Shape;
	 	 	velocity : Box2D.Common.Math.b2Vec2;
		}
		export class b2DistanceInput{
			proxyA : b2DistanceProxy;
	 	 	proxyB : b2DistanceProxy;
	 	 	transformA : Box2D.Common.Math.b2Transform;
	 	 	transformB : Box2D.Common.Math.b2Transform;
	 	 	useRadii : bool;
		}
		export class b2DistanceOutput{
	 	 	distance : number;
	 	 	iterations : number;
	 	 	pointA : Box2D.Common.Math.b2Vec2;
	 	 	pointB : Box2D.Common.Math.b2Vec2;
		}
		export class b2DistanceProxy{
			m_count : number;
	 	 	m_radius : number;
	 	 	m_vertices : Box2D.Common.Math.b2Vec2[];
			GetSupport(d:Box2D.Common.Math.b2Vec2):number;
			GetSupportVertex(d:Box2D.Common.Math.b2Vec2):Box2D.Common.Math.b2Vec2;
			GetVertex(index:number):Box2D.Common.Math.b2Vec2;
			GetVertexCount():number;
			Set(shape:Box2D.Collision.Shapes.b2Shape):void;
		}
		export class b2DynamicTree{
			constructor();
			CreateProxy(aabb:Box2D.Collision.b2AABB, userData:any):b2DynamicTreeNode;
			DestroyProxy(proxy:b2DynamicTreeNode):void;
			GetFatAABB(proxy:b2DynamicTreeNode):Box2D.Collision.b2AABB;
			GetUserData(proxy:b2DynamicTreeNode):any;
			MoveProxy(proxy:b2DynamicTreeNode, aabb:Box2D.Collision.b2AABB, displacement:Box2D.Common.Math.b2Vec2):bool;
			Query(callback:(proxy:b2DynamicTreeNode)=>bool, aabb:Box2D.Collision.b2AABB):void;
			RayCast(callback:(input:b2RayCastInput, proxy:any)=>void, input:Box2D.Collision.b2RayCastInput):void;
			Rebalance(iterations:number):void;
		}
		export class b2DynamicTreeNode{

		}
		export class b2DynamicTreeBroadPhase{
			CreateProxy(aabb:Box2D.Collision.b2AABB, userData:any):any;
			DestroyProxy(proxy:any):void;
			GetFatAABB(proxy:any):Box2D.Collision.b2AABB;
			GetProxyCount():number;
			GetUserData(proxy:any):any;
			MoveProxy(proxy:any, aabb:Box2D.Collision.b2AABB, displacement:Box2D.Common.Math.b2Vec2):void;
			Query(callback:(proxy:b2DynamicTreeNode)=>bool, aabb:Box2D.Collision.b2AABB):void;
			RayCast(callback:(proxy:b2DynamicTreeNode)=>bool, input:Box2D.Collision.b2RayCastInput):void;
			Rebalance(iterations:number):void;
			TestOverlap(proxyA:any, proxyB:any):bool;
			UpdatePairs(callback:(userDataA:any, userDataB:any)=>void):void;
			Validate():void;
		}
		export class b2Manifold{
			m_localPlaneNormal : Box2D.Common.Math.b2Vec2;
	 	 	m_localPoint : Box2D.Common.Math.b2Vec2;
	 	 	m_pointCount : number;
	 	 	m_points : b2ManifoldPoint[];
	 	 	m_type : number;
	 	 	constructor();
			Copy():b2Manifold;
			Reset():void;
			Set(m:b2Manifold):void;
	 	 	static e_circles : number;
	 	 	static e_faceA : number;
	 	 	static e_faceB : number;
		}
		export class b2ManifoldPoint{
			m_id : b2ContactID;
	 	 	m_localPoint : Box2D.Common.Math.b2Vec2;
	 	 	m_normalImpulse : number;
	 	 	m_tangentImpulse : number;
			constructor();
			Reset():void;
			Set(m:b2ManifoldPoint):void;
		}
		export class b2OBB{
			center : Box2D.Common.Math.b2Vec2;
	 	 	extents : Box2D.Common.Math.b2Vec2;
	 	 	R : Box2D.Common.Math.b2Mat22;
		}
		export class b2RayCastInput{
			maxFraction : number;
	 	 	p1 : Box2D.Common.Math.b2Vec2;
	 	 	p2 : Box2D.Common.Math.b2Vec2;
			constructor(p1:Box2D.Common.Math.b2Vec2, p2:Box2D.Common.Math.b2Vec2, maxFraction:number);
		}
		export class b2RayCastOutput{
			fraction : number;
 	 		normal : Box2D.Common.Math.b2Vec2;
		}
		export class b2Segment{
			p1 : Box2D.Common.Math.b2Vec2;
			p2 : Box2D.Common.Math.b2Vec2;
			Extend(aabb:Box2D.Collision.b2AABB):void;
			ExtendBackward(aabb:Box2D.Collision.b2AABB):void;
			ExtendForward(aabb:Box2D.Collision.b2AABB):void;
			TestSegment(lambda:Array, normal:Box2D.Common.Math.b2Vec2, segment:b2Segment, maxLambda:number):bool;
		}
		export class b2SimplexCache{
			count : number;
	 	 	indexA : number[];
	 	 	indexB : number[];
	 	 	metric : number;
		}
		export class b2TOIInput{
			proxyA : b2DistanceProxy;
		 	proxyB : b2DistanceProxy;
		 	sweepA : Box2D.Common.Math.b2Sweep;
		 	sweepB : Box2D.Common.Math.b2Sweep;
		 	tolerance : number;
		}
		export class b2WorldManifold{
			m_normal : Box2D.Common.Math.b2Vec2;
			m_points : Box2D.Common.Math.b2Vec2[];
			constructor();
			Initialize(manifold:Box2D.Collision.b2Manifold, xfA:Box2D.Common.Math.b2Transform, radiusA:number, xfB:Box2D.Common.Math.b2Transform, radiusB:number):void;
		}
		export class Features{
			flip : number;
	 	 	incidentEdge : number;
	 	 	incidentVertex : number;
	 	 	referenceEdge : number;
		}
		export module Shapes{
			export class b2Shape{
				ComputeAABB(aabb:Box2D.Collision.b2AABB, xf:Box2D.Common.Math.b2Transform):void;
				ComputeMass(massData:Box2D.Collision.Shapes.b2MassData, density:number):void;
				ComputeSubmergedArea(normal:Box2D.Common.Math.b2Vec2, offset:number, xf:Box2D.Common.Math.b2Transform, c:Box2D.Common.Math.b2Vec2):number;
				Copy():Box2D.Collision.Shapes.b2Shape;
				GetType():number;
				RayCast(output:Box2D.Collision.b2RayCastOutput, input:Box2D.Collision.b2RayCastInput, transform:Box2D.Common.Math.b2Transform):bool;
				Set(other:Box2D.Collision.Shapes.b2Shape):void;
				static TestOverlap(shape1:Box2D.Collision.Shapes.b2Shape, transform1:Box2D.Common.Math.b2Transform, shape2:Box2D.Collision.Shapes.b2Shape, transform2:Box2D.Common.Math.b2Transform):bool;
				TestPoint(xf:Box2D.Common.Math.b2Transform, p:Box2D.Common.Math.b2Vec2):bool;
				static e_hitCollide : number;
				static e_missCollide : number;
				static e_startsInsideCollide : number;
			}
			export class b2CircleShape extends b2Shape{
				constructor(radius:number);
				ComputeAABB(aabb:Box2D.Collision.b2AABB, transform:Box2D.Common.Math.b2Transform):void;
				ComputeMass(massData:Box2D.Collision.Shapes.b2MassData, density:number):void;
				ComputeSubmergedArea(normal:Box2D.Common.Math.b2Vec2, offset:number, xf:Box2D.Common.Math.b2Transform, c:Box2D.Common.Math.b2Vec2):number;
				Copy():Box2D.Collision.Shapes.b2Shape;
				GetLocalPosition():Box2D.Common.Math.b2Vec2;
				GetRadius():number;
				RayCast(output:Box2D.Collision.b2RayCastOutput, input:Box2D.Collision.b2RayCastInput, transform:Box2D.Common.Math.b2Transform):bool;
				Set(other:Box2D.Collision.Shapes.b2Shape):void;
				SetLocalPosition(position:Box2D.Common.Math.b2Vec2):void;
				SetRadius(radius:number):void;
				TestPoint(transform:Box2D.Common.Math.b2Transform, p:Box2D.Common.Math.b2Vec2):bool;
			}
			export class b2EdgeChainDef{
				isALoop : bool;
		 	 	vertexCount : number;
		 	 	vertices : Array;
				constructor();
			}
			export class b2MassData{
				center : Box2D.Common.Math.b2Vec2;
		 	 	I : number;
		 	 	mass : number;
			}
			export class b2PolygonShape extends b2Shape{
				static AsArray(vertices:Box2D.Common.Math.b2Vec2[], vertexCount:number):b2PolygonShape;
				static AsBox(hx:number, hy:number):b2PolygonShape;
				static AsEdge(v1:Box2D.Common.Math.b2Vec2, v2:Box2D.Common.Math.b2Vec2):b2PolygonShape;
				static AsOrientedBox(hx:number, hy:number, center:Box2D.Common.Math.b2Vec2, angle:number):b2PolygonShape;
				static AsVector(vertices:Box2D.Common.Math.b2Vec2[], vertexCount:number):b2PolygonShape;
				ComputeAABB(aabb:Box2D.Collision.b2AABB, xf:Box2D.Common.Math.b2Transform):void;
				static ComputeCentroid(vs:Box2D.Common.Math.b2Vec2[], count:number):Box2D.Common.Math.b2Vec2;
				staticComputeMass(massData:Box2D.Collision.Shapes.b2MassData, density:number):void;
				ComputeSubmergedArea(normal:Box2D.Common.Math.b2Vec2, offset:number, xf:Box2D.Common.Math.b2Transform, c:Box2D.Common.Math.b2Vec2):number;
				Copy():Box2D.Collision.Shapes.b2Shape;
				GetNormals():Box2D.Common.Math.b2Vec2[];
				GetSupport(d:Box2D.Common.Math.b2Vec2):number;
				GetSupportVertex(d:Box2D.Common.Math.b2Vec2):Box2D.Common.Math.b2Vec2;
				GetVertexCount():number;
				GetVertices():Box2D.Common.Math.b2Vec2[];
				RayCast(output:Box2D.Collision.b2RayCastOutput, input:Box2D.Collision.b2RayCastInput, transform:Box2D.Common.Math.b2Transform):bool;
				Set(other:Box2D.Collision.Shapes.b2Shape):void;
				SetAsArray(vertices:Array, vertexCount:number):void;
				SetAsBox(hx:number, hy:number):void;
				SetAsEdge(v1:Box2D.Common.Math.b2Vec2, v2:Box2D.Common.Math.b2Vec2):void;
				SetAsOrientedBox(hx:number, hy:number, center:Box2D.Common.Math.b2Vec2, angle:number):void;
				SetAsVector(vertices:Box2D.Common.Math.b2Vec2[], vertexCount:number):void;
				TestPoint(xf:Box2D.Common.Math.b2Transform, p:Box2D.Common.Math.b2Vec2):bool;
			}

		}
	}
	export module Common{
		export module Math{
			export class b2Vec2{
		 	 	x : number;
		 	 	y : number;
				constructor(x_:number, y_:number);
				Abs() : void;
				Add(v:Box2D.Common.Math.b2Vec2) : void;
				Copy() : Box2D.Common.Math.b2Vec2;
				CrossFV(s:number):void;
				CrossVF(s:number):void;
				GetNegative() : Box2D.Common.Math.b2Vec2;
				IsValid():bool;
				Length():number;
				LengthSquared():number;
				static Make(x_:number, y_:number):Box2D.Common.Math.b2Vec2;
				MaxV(b:Box2D.Common.Math.b2Vec2):void;
				MinV(b:Box2D.Common.Math.b2Vec2):void;
				MulM(A:Box2D.Common.Math.b2Mat22):void;
				Multiply(a:number):void;
				MulTM(A:Box2D.Common.Math.b2Mat22):void;
				NegativeSelf():void;
				Normalize():number;
				Set(x_:number, y_:number):void;
				SetV(v:Box2D.Common.Math.b2Vec2):void; 	 	
				SetZero():void;
				Subtract(v:Box2D.Common.Math.b2Vec2):void;
			}

			export class b2Vec3{
				x : number;
		 	 	y : number;
		 	 	z : number;
				constructor(x:number, y:number, z:number);
				Add(v:Box2D.Common.Math.b2Vec3):void;
				Copy():Box2D.Common.Math.b2Vec3;
				GetNegative():Box2D.Common.Math.b2Vec3;
				Multiply(a:number):void;
				NegativeSelf():void;
				Set(x:number, y:number, z:number):void;
				SetV(v:Box2D.Common.Math.b2Vec3):void;
				SetZero():void;
				Subtract(v:Box2D.Common.Math.b2Vec3):void;
			}

			export class b2Mat22{
		 	 	col1 : Box2D.Common.Math.b2Vec2;
		 	 	col2 : Box2D.Common.Math.b2Vec2;
				constructor();
				Abs():void;
				AddM(m:Box2D.Common.Math.b2Mat22):void;
				Copy():Box2D.Common.Math.b2Mat22;
				static FromAngle(angle:number):Box2D.Common.Math.b2Mat22;
				static FromVV(c1:Box2D.Common.Math.b2Vec2, c2:Box2D.Common.Math.b2Vec2):Box2D.Common.Math.b2Mat22;
				GetAngle():number;
				GetInverse(out:Box2D.Common.Math.b2Mat22):Box2D.Common.Math.b2Mat22;
				Set(angle:number):void;
				SetIdentity():void;
				SetM(m:Box2D.Common.Math.b2Mat22):void;
				SetVV(c1:Box2D.Common.Math.b2Vec2, c2:Box2D.Common.Math.b2Vec2):void;
				SetZero():void;
				Solve(out:Box2D.Common.Math.b2Vec2, bX:number, bY:number):Box2D.Common.Math.b2Vec2;
			}

			export class b2Mat33{
				col1 : Box2D.Common.Math.b2Vec3;
		 	 	col2 : Box2D.Common.Math.b2Vec3;
		 	 	col3 : Box2D.Common.Math.b2Vec3;
				constructor(c1:Box2D.Common.Math.b2Vec3, c2:Box2D.Common.Math.b2Vec3, c3:Box2D.Common.Math.b2Vec3);
				AddM(m:b2Mat33):void;
				Copy():b2Mat33;
				SetIdentity():void;
				SetM(m:b2Mat33):void;
				SetVVV(c1:Box2D.Common.Math.b2Vec3, c2:Box2D.Common.Math.b2Vec3, c3:Box2D.Common.Math.b2Vec3):void;
				SetZero():void;
				Solve22(out:Box2D.Common.Math.b2Vec2, bX:number, bY:number):Box2D.Common.Math.b2Vec2;
				Solve33(out:Box2D.Common.Math.b2Vec3, bX:number, bY:number, bZ:number):Box2D.Common.Math.b2Vec3;
			}

			export class b2Sweep{
				a : number;
		 	 	a0 : number;
		 	 	c : Box2D.Common.Math.b2Vec2;
		 	 	c0 : Box2D.Common.Math.b2Vec2;
		 	 	localCenter : Box2D.Common.Math.b2Vec2;
		 	 	t0 : number;
			}

			export class b2Transform{
		 	 	position : Box2D.Common.Math.b2Vec2;
		 	 	R : Box2D.Common.Math.b2Mat22;
				constructor(pos:Box2D.Common.Math.b2Vec2, r:Box2D.Common.Math.b2Mat22);
				GetAngle():number;
				Initialize(pos:Box2D.Common.Math.b2Vec2, r:Box2D.Common.Math.b2Mat22):void;
				Set(x:Box2D.Common.Math.b2Transform):void;
				SetIdentity():void;
			}
		}

		export class b2Color{
			b : number;
	 	 	color : number;
	 	 	g : number;
	 	 	r : number;
			constructor(rr:number, gg:number, bb:number);
			Set(rr:number, gg:number, bb:number):void;
		}	

		export class b2Settings{
			static b2Assert(a:bool):void;
			static b2MixFriction(friction1:number, friction2:number):number;
			static b2MixRestitution(restitution1:number, restitution2:number):number;
	 	 	static b2_aabbExtension : number;
	 	 	static b2_aabbMultiplier : number;
	 	 	static b2_angularSleepTolerance : number;
	 	 	static b2_angularSlop : number;
	 	 	static b2_contactBaumgarte : number;
	 	 	static b2_linearSleepTolerance : number;
	 	 	static b2_linearSlop : number;
	 	 	static b2_maxAngularCorrection : number;
	 	 	static b2_maxLinearCorrection : number;
	 	 	static b2_maxManifoldPoints : number;
	 	 	static b2_maxRotation : number;
	 	 	static b2_maxRotationSquared : number;
	 	 	static b2_maxTOIContactsPerIsland : number;
	 	 	static b2_maxTOIJointsPerIsland : number;
	 	 	static b2_maxTranslation : number;
	 	 	static b2_maxTranslationSquared : number;
	 	 	static b2_pi : number;
	 	 	static b2_polygonRadius : number;
	 	 	static b2_timeToSleep : number;
	 	 	static b2_toiSlop : number;
	 	 	static b2_velocityThreshold : number;
	 	 	static USHRT_MAX : number;
	 	 	static VERSION : string;
		}
	}

	export module Dynamics{
		export class b2Body{
			static b2_dynamicBody : number;
	 	 	static b2_kinematicBody : number;
	 	 	static b2_staticBody : number;
			ApplyForce(force:Box2D.Common.Math.b2Vec2, point:Box2D.Common.Math.b2Vec2):void;
			ApplyImpulse(impulse:Box2D.Common.Math.b2Vec2, point:Box2D.Common.Math.b2Vec2):void;
			ApplyTorque(torque:number):void;
			CreateFixture(def:Box2D.Dynamics.b2FixtureDef):Box2D.Dynamics.b2Fixture;
			CreateFixture2(shape:Box2D.Collision.Shapes.b2Shape, density:number):Box2D.Dynamics.b2Fixture;
			DestroyFixture(fixture:Box2D.Dynamics.b2Fixture):void;
			GetAngle():number;
			GetAngularDamping():number;
			GetAngularVelocity():number;
			GetContactList():Box2D.Dynamics.Contacts.b2ContactEdge;
			GetControllerList():Box2D.Dynamics.Controllers.b2ControllerEdge;
			GetDefinition():b2BodyDef;
			GetFixtureList():Box2D.Dynamics.b2Fixture;
			GetInertia():number;
			GetJointList():Box2D.Dynamics.Joints.b2JointEdge;
			GetLinearDamping():number;
			GetLinearVelocity():Box2D.Common.Math.b2Vec2;
			GetLinearVelocityFromLocalPoint(localPoint:Box2D.Common.Math.b2Vec2):Box2D.Common.Math.b2Vec2;
			GetLinearVelocityFromWorldPoint(worldPoint:Box2D.Common.Math.b2Vec2):Box2D.Common.Math.b2Vec2;
			GetLocalCenter():Box2D.Common.Math.b2Vec2;
			GetLocalPoint(worldPoint:Box2D.Common.Math.b2Vec2):Box2D.Common.Math.b2Vec2;
			GetLocalVector(worldVector:Box2D.Common.Math.b2Vec2):Box2D.Common.Math.b2Vec2;
			GetMass():number;
			GetMassData(data:Box2D.Collision.Shapes.b2MassData):void;
			GetNext():Box2D.Dynamics.b2Body;
			GetPosition():Box2D.Common.Math.b2Vec2;
			GetTransform():Box2D.Common.Math.b2Transform;
			GetType():number;
			GetUserData(): any; 	 	
			GetWorld():b2World;
			GetWorldCenter():Box2D.Common.Math.b2Vec2;
			GetWorldPoint(localPoint:Box2D.Common.Math.b2Vec2):Box2D.Common.Math.b2Vec2;
			GetWorldVector(localVector:Box2D.Common.Math.b2Vec2):Box2D.Common.Math.b2Vec2;
			IsActive():bool;
			IsAwake():bool;
			IsBullet():bool;
			IsFixedRotation():bool;
			IsSleepingAllowed():bool;
			Merge(other:Box2D.Dynamics.b2Body):void;
			ResetMassData():void;
			SetActive(flag:bool):void;
			SetAngle(angle:number):void;
			SetAngularDamping(angularDamping:number):void;
			SetAngularVelocity(omega:number):void;
			SetAwake(flag:bool):void;
			SetBullet(flag:bool):void;
			SetFixedRotation(fixed:bool):void;
			SetLinearDamping(linearDamping:number):void;
			SetLinearVelocity(v:Box2D.Common.Math.b2Vec2):void;
			SetMassData(massData:Box2D.Collision.Shapes.b2MassData):void;
			SetPosition(position:Box2D.Common.Math.b2Vec2):void;
			SetPositionAndAngle(position:Box2D.Common.Math.b2Vec2, angle:number):void;
			SetSleepingAllowed(flag:bool):void;
			SetTransform(xf:Box2D.Common.Math.b2Transform):void;
			SetType(type:number):void;
			SetUserData(data:any):void;
			Split(callback:(fixture:b2Fixture)=>bool):Box2D.Dynamics.b2Body;
		}

		export class b2BodyDef{
			active : bool;
		 	allowSleep : bool;
	 	 	angle : number;
	 	 	angularDamping : number;
	 	 	angularVelocity : number;
	 	 	awake : bool;
	 	 	bullet : bool;
	 	 	fixedRotation : bool;
	 	 	inertiaScale : number;
	 	 	linearDamping : number;
	 	 	linearVelocity : Box2D.Common.Math.b2Vec2;
	 	 	position : Box2D.Common.Math.b2Vec2;
	 	 	type : number;
	 	 	userData : any;
			constructor();
		}

		export interface b2ContactFilter{
			RayCollide(userData:any, fixture:b2Fixture):bool;
			ShouldCollide(fixtureA:b2Fixture, fixtureB:b2Fixture):bool;
		}

		export class b2ContactImpulse{
			normalImpulses : number[];
	 	 	tangentImpulses : number[];
		}

		export interface b2ContactListener{
			BeginContact(contact:Box2D.Dynamics.Contacts.b2Contact):void;
			EndContact(contact:Box2D.Dynamics.Contacts.b2Contact):void;
			PostSolve(contact:Box2D.Dynamics.Contacts.b2Contact, impulse:b2ContactImpulse):void;
			PreSolve(contact:Box2D.Dynamics.Contacts.b2Contact, oldManifold:Box2D.Collision.b2Manifold):void;
		}

		export class b2DebugDraw{
			static e_aabbBit : number;
	 	 	static e_centerOfMassBit : number;
	 	 	static e_controllerBit : number;
	 	 	static e_jointBit : number;
 	 		static e_pairBit : number;
	 	 	static e_shapeBit : number;
			constructor();
			AppendFlags(flags:number):void;
			ClearFlags(flags:number):void;
			DrawCircle(center:Box2D.Common.Math.b2Vec2, radius:number, color:Box2D.Common.b2Color):void;
			DrawPolygon(vertices:Box2D.Common.Math.b2Vec2[], vertexCount:number, color:Box2D.Common.b2Color):void;
			DrawSegment(p1:Box2D.Common.Math.b2Vec2, p2:Box2D.Common.Math.b2Vec2, color:Box2D.Common.b2Color):void;
			DrawSolidCircle(center:Box2D.Common.Math.b2Vec2, radius:number, axis:Box2D.Common.Math.b2Vec2, color:Box2D.Common.b2Color):void;
			DrawSolidPolygon(vertices:Box2D.Common.Math.b2Vec2[], vertexCount:number, color:Box2D.Common.b2Color):void;
			DrawTransform(xf:Box2D.Common.Math.b2Transform):void;
			GetAlpha():number;
			GetDrawScale():number;
			GetFillAlpha():number;
			GetFlags():number;
			GetLineThickness():number;
			GetSprite():CanvasRenderingContext2D;
			GetXFormScale():number;
			SetAlpha(alpha:Number):void;
			SetDrawScale(drawScale:number):void;
			SetFillAlpha(alpha:number):void;
			SetFlags(flags:number):void;
			SetLineThickness(lineThickness:number):void;
			SetSprite(sprite:CanvasRenderingContext2D):void;
			SetXFormScale(xformScale:number):void;
		}

		export class b2DestructionListener{
			SayGoodbyeFixture(fixture:b2Fixture):void;
			SayGoodbyeJoint(joint:Box2D.Dynamics.Joints.b2Joint):void;
		}

		export class b2FilterData{
			categoryBits : number;
	 	 	groupIndex : number ;
	 	 	maskBits : number;
		}

		export class b2Fixture{
			GetAABB():Box2D.Collision.b2AABB;
			GetBody():Box2D.Dynamics.b2Body;
			GetDensity():number;
			GetFilterData():b2FilterData;
			GetFriction():number;
			GetMassData(massData:Box2D.Collision.Shapes.b2MassData):Box2D.Collision.Shapes.b2MassData;
			GetNext():Box2D.Dynamics.b2Fixture;
			GetRestitution():number;
			GetShape():Box2D.Collision.Shapes.b2Shape;
			GetType():number;
			GetUserData():any;
			IsSensor():bool;
			RayCast(output:Box2D.Collision.b2RayCastOutput, input:Box2D.Collision.b2RayCastInput):bool;
			SetDensity(density:number):void;
			SetFilterData(filter:b2FilterData):void;
			SetFriction(friction:number):void;
			SetRestitution(restitution:number):void;
			SetSensor(sensor:bool):void;
			SetUserData(data:any):void;
			TestPoint(p:Box2D.Common.Math.b2Vec2):bool;
		}

		export class b2FixtureDef{
			density : number;
	 	 	filter : b2FilterData;
	 	 	friction : number;
	 	 	isSensor : bool;
	 	 	restitution : number;
	 	 	shape : Box2D.Collision.Shapes.b2Shape;
	 	 	userData : any;
			constructor();
		}

		export class b2TimeStep{
			dt : number;
 	 		inv_dt : number;
 	 		iterations : number;
		}

		export class b2World{
			constructor(gravity:Box2D.Common.Math.b2Vec2, doSleep:bool);
			AddController(c:Box2D.Dynamics.Controllers.b2Controller):Box2D.Dynamics.Controllers.b2Controller;
			ClearForces():void;
			CreateBody(def:b2BodyDef):Box2D.Dynamics.b2Body;
			CreateController(controller:Box2D.Dynamics.Controllers.b2Controller):Box2D.Dynamics.Controllers.b2Controller;
			CreateJoint(def:Box2D.Dynamics.Joints.b2JointDef):Box2D.Dynamics.Joints.b2Joint;
			DestroyBody(b:Box2D.Dynamics.b2Body):void;
			DestroyController(controller:Box2D.Dynamics.Controllers.b2Controller):void;
			DestroyJoint(j:Box2D.Dynamics.Joints.b2Joint):void;
			DrawDebugData():void;
			GetBodyCount():number;
			GetBodyList():Box2D.Dynamics.b2Body;
			GetContactCount():number;
			GetContactList():Box2D.Dynamics.Contacts.b2Contact;
			GetGravity():Box2D.Common.Math.b2Vec2;
			GetGroundBody():Box2D.Dynamics.b2Body;
			GetJointCount():number;
			GetJointList():Box2D.Dynamics.Joints.b2Joint;
			GetProxyCount():number;
			IsLocked():bool;
			QueryAABB(callback:(fixture:b2Fixture)=>bool, aabb:Box2D.Collision.b2AABB):void;
			QueryPoint(callback:(fixture:b2Fixture)=>bool, p:Box2D.Common.Math.b2Vec2):void;
			QueryShape(callback:(fixture:b2Fixture)=>bool, shape:Box2D.Collision.Shapes.b2Shape, transform:Box2D.Common.Math.b2Transform):void;
			RayCast(callback:(fixture:b2Fixture, point:Box2D.Common.Math.b2Vec2, normal:Box2D.Common.Math.b2Vec2, fraction:number)=>number, point1:Box2D.Common.Math.b2Vec2, point2:Box2D.Common.Math.b2Vec2):void;
			RayCastAll(point1:Box2D.Common.Math.b2Vec2, point2:Box2D.Common.Math.b2Vec2):b2Fixture[];
			RayCastOne(point1:Box2D.Common.Math.b2Vec2, point2:Box2D.Common.Math.b2Vec2):Box2D.Dynamics.b2Fixture;
			RemoveController(c:Box2D.Dynamics.Controllers.b2Controller):void;
			SetBroadPhase(broadPhase:Box2D.Collision.IBroadPhase):void;
			SetContactFilter(filter:b2ContactFilter):void;
			SetContactListener(listener:b2ContactListener):void;
			SetContinuousPhysics(flag:bool):void;
			SetDebugDraw(debugDraw:b2DebugDraw):void;
			SetDestructionListener(listener:b2DestructionListener):void;
			SetGravity(gravity:Box2D.Common.Math.b2Vec2):void;
			SetWarmStarting(flag:bool):void;
			Step(dt:number, velocityIterations:number, positionIterations:number):void;
			Validate():void;
			static e_locked : number;
			static e_newFixture : number;
		}

		export module Contacts{
			export class b2Contact{
				constructor();
				FlagForFiltering():void;
				GetFixtureA():b2Fixture;
				GetFixtureB():b2Fixture;
				GetManifold():Box2D.Collision.b2Manifold;
				GetNext():b2Contact;
				GetWorldManifold(worldManifold:Box2D.Collision.b2WorldManifold):void;
				IsContinuous():bool;
				IsEnabled():bool;
				IsSensor():bool;
				IsTouching():bool;
				SetEnabled(flag:bool):void;
				SetSensor(sensor:bool):void;
			}
			export class b2ContactEdge{
				contact : b2Contact;
		 	 	next : b2ContactEdge;
		 	 	other : b2Body;
 			 	prev : b2ContactEdge;
			}
			export class b2ContactResult{
				id : Box2D.Collision.b2ContactID;
		 	 	normal : Box2D.Common.Math.b2Vec2;
		 	 	normalImpulse : number;
		 	 	position : Box2D.Common.Math.b2Vec2;
		 	 	shape1 : Box2D.Collision.Shapes.b2Shape;
		 	 	shape2 : Box2D.Collision.Shapes.b2Shape;
		 	 	tangentImpulse : number;
			}
		}

		export module Controllers{
			export class b2Controller{
				m_bodyCount : number;
			 	m_bodyList : Box2D.Dynamics.Controllers.b2ControllerEdge;
				AddBody(body:b2Body):void;
				Clear():void;
				Draw(debugDraw:b2DebugDraw):void;
				GetBodyList():Box2D.Dynamics.Controllers.b2ControllerEdge;
				GetNext():b2Controller;
				GetWorld():b2World;
				RemoveBody(body:b2Body):void;
				Step(step:b2TimeStep):void;
			}
			export class b2BuoyancyController extends b2Controller{
				angularDrag : number;
				density : number;
				gravity : Box2D.Common.Math.b2Vec2;
				linearDrag : number;
		 	 	normal : Box2D.Common.Math.b2Vec2;
		 	 	offset : number;
		 	 	useDensity : bool;
		 	 	useWorldGravity : bool;
		 	 	velocity : Box2D.Common.Math.b2Vec2;
			}
			export class b2ConstantAccelController extends b2Controller{
				A : Box2D.Common.Math.b2Vec2;
			}
			export class b2ConstantForceController extends b2Controller{
				F : Box2D.Common.Math.b2Vec2;
			}
			export class b2ControllerEdge{
				body : b2Body;
		 	 	controller : b2Controller;
		 	 	nextBody : b2ControllerEdge;
		 	 	nextController : b2ControllerEdge;
		 	 	prevBody : b2ControllerEdge;
		 	 	prevController : b2ControllerEdge;
			}
			export class b2GravityController extends b2Controller{
				G : number;
		 	 	invSqr : bool;
			}
			export class b2TensorDampingController extends b2Controller{
				maxTimestep : number;
		 	 	T : Box2D.Common.Math.b2Mat22;
				SetAxisAligned(xDamping:number, yDamping:number):void;
			}
		}
		export module Joints{
			export class b2Joint{
				GetAnchorA():Box2D.Common.Math.b2Vec2;				 	 	
				GetAnchorB():Box2D.Common.Math.b2Vec2;
				GetBodyA():b2Body;
				GetBodyB():b2Body;
				GetNext():b2Joint;
				GetReactionForce(inv_dt:number):Box2D.Common.Math.b2Vec2;
				GetReactionTorque(inv_dt:number):number;
				GetType():number;
				GetUserData():any;
				IsActive():bool;
				SetUserData(data:any):void;
			}
			export class b2JointDef{
				bodyA : b2Body;
		 	 	bodyB : b2Body;
		 	 	collideConnected : bool;
		 	 	type : number;
		 	 	userData : any;
				constructor();
			}
			export class b2DistanceJoint extends b2Joint{
				GetDampingRatio():number;
				GetFrequency():number;
				GetLength():number;
				GetReactionForce(inv_dt:number):Box2D.Common.Math.b2Vec2;
				GetReactionTorque(inv_dt:number):number;
				SetDampingRatio(ratio:number):void;
				SetFrequency(hz:number):void;
				SetLength(length:number):void;
			}
			export class b2DistanceJointDef extends b2JointDef{
				dampingRatio : number;
		 	 	frequencyHz : number;
		 	 	length : number;
		 	 	localAnchorA : Box2D.Common.Math.b2Vec2;
		 	 	localAnchorB : Box2D.Common.Math.b2Vec2;
				constructor();
				Initialize(bA:b2Body, bB:b2Body, anchorA:Box2D.Common.Math.b2Vec2, anchorB:Box2D.Common.Math.b2Vec2):void;
			}
			export class b2FrictionJoint extends b2Joint{
				m_angularMass : number;
		 	 	m_linearMass : Box2D.Common.Math.b2Mat22;
				GetAnchorA():Box2D.Common.Math.b2Vec2;
				GetAnchorB():Box2D.Common.Math.b2Vec2;
				GetMaxForce():number;
				GetMaxTorque():number;
				GetReactionForce(inv_dt:number):Box2D.Common.Math.b2Vec2;
				GetReactionTorque(inv_dt:number):number;
				SetMaxForce(force:number):void;
				SetMaxTorque(torque:number):void;
			}
			export class b2FrictionJointDef extends b2JointDef{
				localAnchorA : Box2D.Common.Math.b2Vec2;
		 	 	localAnchorB : Box2D.Common.Math.b2Vec2;
		 	 	maxForce : number;
		 	 	maxTorque : number;
		 	 	constructor();
				Initialize(bA:b2Body, bB:b2Body, anchor:Box2D.Common.Math.b2Vec2):void;
			}
			export class b2GearJoint extends b2Joint{
				GetRatio():number;
				GetReactionForce(inv_dt:number):Box2D.Common.Math.b2Vec2;
				GetReactionTorque(inv_dt:number):number;
				SetRatio(ratio:number):void;
			}
			export class b2GearJointDef extends b2JointDef{
				joint1 : b2Joint;
		 	 	joint2 : b2Joint;
		 	 	ratio : number;
				constructor();
			}

			export class b2JointEdge{
				joint : b2Joint;
		 	 	next : b2JointEdge;
		 	 	other : b2Body;
		 	 	prev : b2JointEdge;
			}
			export class b2LineJoint extends b2Joint{
				EnableLimit(flag:bool):void;
				EnableMotor(flag:bool):void;
				GetJointSpeed():number;
				GetJointTranslation():number;
				GetLowerLimit():number;
				GetMaxMotorForce():number;
				GetMotorForce():number;
				GetMotorSpeed():number;
				GetUpperLimit():number;
				IsLimitEnabled():bool;
				IsMotorEnabled():bool;
				SetLimits(lower:number, upper:number):void;
				SetMaxMotorForce(force:number):void;
				SetMotorSpeed(speed:number):void;
			}
			export class b2LineJointDef extends b2JointDef{
				enableLimit : bool;
		 	 	enableMotor : bool;
		 	 	localAnchorA : Box2D.Common.Math.b2Vec2;
		 	 	localAnchorB : Box2D.Common.Math.b2Vec2;
		 	 	localAxisA : Box2D.Common.Math.b2Vec2;
		 	 	lowerTranslation : number;
		 	 	maxMotorForce : number;
		 	 	motorSpeed : number;
		 	 	upperTranslation : number;
		 	 	constructor();
				Initialize(bA:b2Body, bB:b2Body, anchor:Box2D.Common.Math.b2Vec2, axis:Box2D.Common.Math.b2Vec2):void;
			}
			export class b2MouseJoint extends b2Joint{
				GetDampingRatio():number;
				GetFrequency():number;
				GetMaxForce():number;
				GetReactionForce(inv_dt:number):Box2D.Common.Math.b2Vec2;
				GetReactionTorque(inv_dt:number):number;
				GetTarget():Box2D.Common.Math.b2Vec2;
				SetDampingRatio(ratio:number):void;
				SetFrequency(hz:number):void;
				SetMaxForce(maxForce:number):void;
				SetTarget(target:Box2D.Common.Math.b2Vec2):void;
			}
			export class b2MouseJointDef extends b2JointDef{
				dampingRatio : number;
		 	 	frequencyHz : number;
		 	 	maxForce : number;
		 	 	target : Box2D.Common.Math.b2Vec2;
			}
			export class b2PrismaticJoint extends b2Joint{
				EnableLimit(flag:bool):void;
				EnableMotor(flag:bool):void;
				GetJointSpeed():number;
				GetJointTranslation():number;
				GetLowerLimit():number;
				GetMotorForce():number;
				GetMotorSpeed():number;
				GetReactionForce(inv_dt:number):Box2D.Common.Math.b2Vec2;
				GetReactionTorque(inv_dt:number):number;
				GetUpperLimit():number;
				IsLimitEnabled():bool;
				IsMotorEnabled():bool;
				SetLimits(lower:number, upper:number):void;
				SetMaxMotorForce(force:number):void;
				SetMotorSpeed(speed:number):void;
			}
			export class b2PrismaticJointDef extends b2JointDef{
				enableLimit : bool;
		 	 	enableMotor : bool;
		 	 	localAxisA : Box2D.Common.Math.b2Vec2;
		 	 	lowerTranslation : number;
		 	 	maxMotorForce : number;
		 	 	motorSpeed : number;
		 	 	referenceAngle : number;
		 	 	upperTranslation : number;
				constructor();
				Initialize(bA:b2Body, bB:b2Body, anchor:Box2D.Common.Math.b2Vec2, axis:Box2D.Common.Math.b2Vec2):void;
			}
			export class b2PulleyJoint extends b2Joint{
				GetAnchorA():Box2D.Common.Math.b2Vec2;
				GetAnchorB():Box2D.Common.Math.b2Vec2;
				GetGroundAnchorA():Box2D.Common.Math.b2Vec2;
				GetGroundAnchorB():Box2D.Common.Math.b2Vec2;
				GetLength1():number;
				GetLength2():number;
				GetRatio():number;
				GetReactionForce(inv_dt:number):Box2D.Common.Math.b2Vec2;
				GetReactionTorque(inv_dt:number):number;
			}
			export class b2PulleyJointDef extends b2JointDef{
				groundAnchorA : Box2D.Common.Math.b2Vec2;
		 	 	groundAnchorB : Box2D.Common.Math.b2Vec2;
		 	 	lengthA : number;
		 	 	lengthB : number;
		 	 	localAnchorA : Box2D.Common.Math.b2Vec2;
		 	 	localAnchorB : Box2D.Common.Math.b2Vec2;
		 	 	maxLengthA : number;
		 	 	maxLengthB : number;
		 	 	ratio : number;
		 	 	constructor();
				Initialize(bA:b2Body, bB:b2Body, gaA:Box2D.Common.Math.b2Vec2, gaB:Box2D.Common.Math.b2Vec2, anchorA:Box2D.Common.Math.b2Vec2, anchorB:Box2D.Common.Math.b2Vec2, r:number):void;
			}
			export class b2RevoluteJoint extends b2Joint{
				EnableLimit(flag:bool):void;
				EnableMotor(flag:bool):void;
				GetAnchorA():Box2D.Common.Math.b2Vec2;
				GetAnchorB():Box2D.Common.Math.b2Vec2;
				GetJointAngle():number;
				GetJointSpeed():number;
				GetLowerLimit():number;
				GetMotorSpeed():number;
				GetMotorTorque():number;
				GetReactionForce(inv_dt:number):Box2D.Common.Math.b2Vec2;
				GetReactionTorque(inv_dt:number):number;
				GetUpperLimit():number;
				IsLimitEnabled():bool;
				IsMotorEnabled():bool;
				SetLimits(lower:number, upper:number):void;
				SetMaxMotorTorque(torque:number):void;
				SetMotorSpeed(speed:number):void;
			}
			export class b2RevoluteJointDef extends b2JointDef{
				enableLimit : bool;
		 	 	enableMotor : bool;
		 	 	localAnchorA : Box2D.Common.Math.b2Vec2;
		 	 	localAnchorB : Box2D.Common.Math.b2Vec2;
		 	 	lowerAngle : number;
		 	 	maxMotorTorque : number;
		 	 	motorSpeed : number;
		 	 	referenceAngle : number;
		 	 	upperAngle : number;
				constructor();
				Initialize(bA:b2Body, bB:b2Body, anchor:Box2D.Common.Math.b2Vec2):void;
			}
			export class b2WeldJoint{
				GetAnchorA():Box2D.Common.Math.b2Vec2;
				GetAnchorB():Box2D.Common.Math.b2Vec2;
				GetReactionForce(inv_dt:number):Box2D.Common.Math.b2Vec2;
				GetReactionTorque(inv_dt:number):number;
			}
			export class b2WeldJointDef extends b2JointDef{
				localAnchorA : Box2D.Common.Math.b2Vec2;
		 	 	localAnchorB : Box2D.Common.Math.b2Vec2;
		 	 	referenceAngle : number;
				constructor();
				Initialize(bA:b2Body, bB:b2Body, anchor:Box2D.Common.Math.b2Vec2):void;
			}
		}
	}
}

