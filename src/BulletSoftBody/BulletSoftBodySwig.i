%module BulletSoftBody

   ///eAeroModel 
	struct eAeroModel { enum _ {
		V_Point,			///Vertex normals are oriented toward velocity
		V_TwoSided,			///Vertex normals are flipped to match velocity	
		V_TwoSidedLiftDrag, ///Vertex normals are flipped to match velocity and lift and drag forces are applied
		V_OneSided,			///Vertex normals are taken as it is	
		F_TwoSided,			///Face normals are flipped to match velocity
		F_TwoSidedLiftDrag,	///Face normals are flipped to match velocity and lift and drag forces are applied 
		F_OneSided,			///Face normals are taken as it is		
		END
	};};

	///eVSolver : velocities solvers
	struct	eVSolver { enum _ {
		Linear,		///Linear solver
		END
	};};

	///ePSolver : positions solvers
	struct	ePSolver { enum _ {
		Linear,		///Linear solver
		Anchors,	///Anchor solver
		RContacts,	///Rigid contacts solver
		SContacts,	///Soft contacts solver
		END
	};};

	///fCollision
	struct fCollision { enum _ {
		RVSmask	=	0x000f,	///Rigid versus soft mask
		SDF_RS	=	0x0001,	///SDF based rigid vs soft
		CL_RS	=	0x0002, ///Cluster vs convex rigid vs soft

		SVSmask	=	0x0030,	///Rigid versus soft mask		
		VF_SS	=	0x0010,	///Vertex vs face soft vs soft handling
		CL_SS	=	0x0020, ///Cluster vs cluster soft vs soft handling
		CL_SELF =	0x0040, ///Cluster soft body self collision
		/* presets	*/ 
		Default	=	SDF_RS,
		END
	};};

    // Soft Body
	/* Base type	*/ 
	struct	Element
	{
		void*			m_tag;			// User data
		Element() : m_tag(0) {}
	};
	/* Material		*/ 
	struct	Material : Element
	{
		btScalar				m_kLST;			// Linear stiffness coefficient [0,1]
		btScalar				m_kAST;			// Area/Angular stiffness coefficient [0,1]
		btScalar				m_kVST;			// Volume stiffness coefficient [0,1]
		int						m_flags;		// Flags
	};

	/* Feature		*/ 
	struct	Feature : Element
	{
		Material*				m_material;		// Material
	};
	/* Node			*/ 
	struct	Node : Feature
	{
		btVector3				m_x;			// Position
		btVector3				m_q;			// Previous step position
		btVector3				m_v;			// Velocity
		btVector3				m_f;			// Force accumulator
		btVector3				m_n;			// Normal
		btScalar				m_im;			// 1/mass
		btScalar				m_area;			// Area
		btDbvtNode*				m_leaf;			// Leaf data
		int						m_battach:1;	// Attached
	};

	/* Link			*/ 
	struct	Link : Feature
	{
		Node*					m_n[2];			// Node pointers
		btScalar				m_rl;			// Rest length		
		int						m_bbending:1;	// Bending link
		btScalar				m_c0;			// (ima+imb)*kLST
		btScalar				m_c1;			// rl^2
		btScalar				m_c2;			// |gradient|^2/c0
		btVector3				m_c3;			// gradient
	};
	/* Face			*/ 
	struct	Face : Feature
	{
		Node*					m_n[3];			// Node pointers
		btVector3				m_normal;		// Normal
		btScalar				m_ra;			// Rest area
		btDbvtNode*				m_leaf;			// Leaf data
	};

	struct	Cluster
	{
		tScalarArray				m_masses;
		btAlignedObjectArray<Node*>	m_nodes;		
		tVector3Array				m_framerefs;
		btTransform					m_framexform;
		btScalar					m_idmass;
		btScalar					m_imass;
		btMatrix3x3					m_locii;
		btMatrix3x3					m_invwi;
		btVector3					m_com;
		btVector3					m_vimpulses[2];
		btVector3					m_dimpulses[2];
		int							m_nvimpulses;
		int							m_ndimpulses;
		btVector3					m_lv;
		btVector3					m_av;
		btDbvtNode*					m_leaf;
		btScalar					m_ndamping;	/* Node damping		*/ 
		btScalar					m_ldamping;	/* Linear damping	*/ 
		btScalar					m_adamping;	/* Angular damping	*/ 
		btScalar					m_matching;
		btScalar					m_maxSelfCollisionImpulse;
		btScalar					m_selfCollisionImpulseFactor;
		bool						m_containsAnchor;
		bool						m_collide;
		int							m_clusterIndex;
		Cluster() : m_leaf(0),m_ndamping(0),m_ldamping(0),m_adamping(0),m_matching(0) 
		,m_maxSelfCollisionImpulse(100.f),
		m_selfCollisionImpulseFactor(0.01f),
		m_containsAnchor(false)
		{}
	};

	struct	Impulse
	{
		btVector3					m_velocity;
		btVector3					m_drift;
		int							m_asVelocity:1;
		int							m_asDrift:1;
		Impulse() : m_velocity(0,0,0),m_drift(0,0,0),m_asVelocity(0),m_asDrift(0)	{}
		Impulse						operator -() const
		{
			Impulse i=*this;
			i.m_velocity=-i.m_velocity;
			i.m_drift=-i.m_drift;
			return(i);
		}
		Impulse						operator*(btScalar x) const
		{
			Impulse i=*this;
			i.m_velocity*=x;
			i.m_drift*=x;
			return(i);
		}
	};

	struct	Body
	{
		Cluster*			m_soft;
		btRigidBody*		m_rigid;
		const btCollisionObject*	m_collisionObject;

		Body() : m_soft(0),m_rigid(0),m_collisionObject(0)				{}
		Body(Cluster* p) : m_soft(p),m_rigid(0),m_collisionObject(0)	{}
		Body(const btCollisionObject* colObj) : m_soft(0),m_collisionObject(colObj)
		{
			m_rigid = (btRigidBody*)btRigidBody::upcast(m_collisionObject);
		}

		void						activate() const
		{
			if(m_rigid) 
				m_rigid->activate();
			if (m_collisionObject)
				m_collisionObject->activate();

		}
		const btMatrix3x3&			invWorldInertia() const
		{
			static const btMatrix3x3	iwi(0,0,0,0,0,0,0,0,0);
			if(m_rigid) return(m_rigid->getInvInertiaTensorWorld());
			if(m_soft)	return(m_soft->m_invwi);
			return(iwi);
		}
		btScalar					invMass() const
		{
			if(m_rigid) return(m_rigid->getInvMass());
			if(m_soft)	return(m_soft->m_imass);
			return(0);
		}
		const btTransform&			xform() const
		{
			static const btTransform	identity=btTransform::getIdentity();		
			if(m_collisionObject) return(m_collisionObject->getWorldTransform());
			if(m_soft)	return(m_soft->m_framexform);
			return(identity);
		}
		btVector3					linearVelocity() const
		{
			if(m_rigid) return(m_rigid->getLinearVelocity());
			if(m_soft)	return(m_soft->m_lv);
			return(btVector3(0,0,0));
		}
		btVector3					angularVelocity(const btVector3& rpos) const
		{			
			if(m_rigid) return(btCross(m_rigid->getAngularVelocity(),rpos));
			if(m_soft)	return(btCross(m_soft->m_av,rpos));
			return(btVector3(0,0,0));
		}
		btVector3					angularVelocity() const
		{			
			if(m_rigid) return(m_rigid->getAngularVelocity());
			if(m_soft)	return(m_soft->m_av);
			return(btVector3(0,0,0));
		}
		btVector3					velocity(const btVector3& rpos) const
		{
			return(linearVelocity()+angularVelocity(rpos));
		}
		void						applyVImpulse(const btVector3& impulse,const btVector3& rpos) const
		{
			if(m_rigid)	m_rigid->applyImpulse(impulse,rpos);
			if(m_soft)	btSoftBody::clusterVImpulse(m_soft,rpos,impulse);
		}
		void						applyDImpulse(const btVector3& impulse,const btVector3& rpos) const
		{
			if(m_rigid)	m_rigid->applyImpulse(impulse,rpos);
			if(m_soft)	btSoftBody::clusterDImpulse(m_soft,rpos,impulse);
		}		
		void						applyImpulse(const Impulse& impulse,const btVector3& rpos) const
		{
			if(impulse.m_asVelocity)	
			{
//				printf("impulse.m_velocity = %f,%f,%f\n",impulse.m_velocity.getX(),impulse.m_velocity.getY(),impulse.m_velocity.getZ());
				applyVImpulse(impulse.m_velocity,rpos);
			}
			if(impulse.m_asDrift)		
			{
//				printf("impulse.m_drift = %f,%f,%f\n",impulse.m_drift.getX(),impulse.m_drift.getY(),impulse.m_drift.getZ());
				applyDImpulse(impulse.m_drift,rpos);
			}
		}
		void						applyVAImpulse(const btVector3& impulse) const
		{
			if(m_rigid)	m_rigid->applyTorqueImpulse(impulse);
			if(m_soft)	btSoftBody::clusterVAImpulse(m_soft,impulse);
		}
		void						applyDAImpulse(const btVector3& impulse) const
		{
			if(m_rigid)	m_rigid->applyTorqueImpulse(impulse);
			if(m_soft)	btSoftBody::clusterDAImpulse(m_soft,impulse);
		}
		void						applyAImpulse(const Impulse& impulse) const
		{
			if(impulse.m_asVelocity)	applyVAImpulse(impulse.m_velocity);
			if(impulse.m_asDrift)		applyDAImpulse(impulse.m_drift);
		}
		void						applyDCImpulse(const btVector3& impulse) const
		{
			if(m_rigid)	m_rigid->applyCentralImpulse(impulse);
			if(m_soft)	btSoftBody::clusterDCImpulse(m_soft,impulse);
		}
	};

		/* Joint		*/ 
	struct	SBJoint    // modify by linyuan for unity's same name class
	{
		struct eType { enum _ {
			Linear=0,
			Angular,
			Contact
		};};
		struct Specs
		{
			Specs() : erp(1),cfm(1),split(1) {}
			btScalar	erp;
			btScalar	cfm;
			btScalar	split;
		};
		Body						m_bodies[2];
		btVector3					m_refs[2];
		btScalar					m_cfm;
		btScalar					m_erp;
		btScalar					m_split;
		btVector3					m_drift;
		btVector3					m_sdrift;
		btMatrix3x3					m_massmatrix;
		bool						m_delete;
		virtual						~SBJoint() {}
		SBJoint() : m_delete(false) {}
		virtual void				Prepare(btScalar dt,int iterations);
		virtual void				Solve(btScalar dt,btScalar sor)=0;
		virtual void				Terminate(btScalar dt)=0;
		virtual eType::_			Type() const=0;
	};
	/* LJoint		*/ 
	struct	LJoint : SBJoint
	{
		struct Specs : SBJoint::Specs
		{
			btVector3	position;
		};		
		btVector3					m_rpos[2];
		void						Prepare(btScalar dt,int iterations);
		void						Solve(btScalar dt,btScalar sor);
		void						Terminate(btScalar dt);
		eType::_					Type() const { return(eType::Linear); }
	};
	/* AJoint		*/ 
	struct	AJoint : SBJoint
	{
		struct IControl
		{
			virtual void			Prepare(AJoint*)				{}
			virtual btScalar		Speed(AJoint*,btScalar current) { return(current); }
			static IControl*		Default()						{ static IControl def;return(&def); }
		};
		struct Specs : SBJoint::Specs
		{
			Specs() : icontrol(IControl::Default()) {}
			btVector3	axis;
			IControl*	icontrol;
		};		
		btVector3					m_axis[2];
		IControl*					m_icontrol;
		void						Prepare(btScalar dt,int iterations);
		void						Solve(btScalar dt,btScalar sor);
		void						Terminate(btScalar dt);
		eType::_					Type() const { return(eType::Angular); }
	};
	/* CJoint		*/ 
	struct	CJoint : SBJoint
	{		
		int							m_life;
		int							m_maxlife;
		btVector3					m_rpos[2];
		btVector3					m_normal;
		btScalar					m_friction;
		void						Prepare(btScalar dt,int iterations);
		void						Solve(btScalar dt,btScalar sor);
		void						Terminate(btScalar dt);
		eType::_					Type() const { return(eType::Contact); }
	};

	struct	Config
	{
		eAeroModel::_			aeromodel;		// Aerodynamic model (default: V_Point)
		btScalar				kVCF;			// Velocities correction factor (Baumgarte)
		btScalar				kDP;			// Damping coefficient [0,1]
		btScalar				kDG;			// Drag coefficient [0,+inf]
		btScalar				kLF;			// Lift coefficient [0,+inf]
		btScalar				kPR;			// Pressure coefficient [-inf,+inf]
		btScalar				kVC;			// Volume conversation coefficient [0,+inf]
		btScalar				kDF;			// Dynamic friction coefficient [0,1]
		btScalar				kMT;			// Pose matching coefficient [0,1]		
		btScalar				kCHR;			// Rigid contacts hardness [0,1]
		btScalar				kKHR;			// Kinetic contacts hardness [0,1]
		btScalar				kSHR;			// Soft contacts hardness [0,1]
		btScalar				kAHR;			// Anchors hardness [0,1]
		btScalar				kSRHR_CL;		// Soft vs rigid hardness [0,1] (cluster only)
		btScalar				kSKHR_CL;		// Soft vs kinetic hardness [0,1] (cluster only)
		btScalar				kSSHR_CL;		// Soft vs soft hardness [0,1] (cluster only)
		btScalar				kSR_SPLT_CL;	// Soft vs rigid impulse split [0,1] (cluster only)
		btScalar				kSK_SPLT_CL;	// Soft vs rigid impulse split [0,1] (cluster only)
		btScalar				kSS_SPLT_CL;	// Soft vs rigid impulse split [0,1] (cluster only)
		btScalar				maxvolume;		// Maximum volume ratio for pose
		btScalar				timescale;		// Time scale
		int						viterations;	// Velocities solver iterations
		int						piterations;	// Positions solver iterations
		int						diterations;	// Drift solver iterations
		int						citerations;	// Cluster solver iterations
		int						collisions;		// Collisions flags
		tVSolverArray			m_vsequence;	// Velocity solvers sequence
		tPSolverArray			m_psequence;	// Position solvers sequence
		tPSolverArray			m_dsequence;	// Drift solvers sequence
	};

	struct	RayResultCallback
	{
		btScalar	m_closestHitFraction;
		const btCollisionObject*		m_collisionObject;
		short int	m_collisionFilterGroup;
		short int	m_collisionFilterMask;
		//@BP Mod - Custom flags, currently used to enable backface culling on tri-meshes, see btRaycastCallback.h. Apply any of the EFlags defined there on m_flags here to invoke.
		unsigned int m_flags;

		virtual ~RayResultCallback()
		{
		}
		bool	hasHit() const
		{
			return (m_collisionObject != 0);
		}

		RayResultCallback()
			:m_closestHitFraction(btScalar(1.)),
			m_collisionObject(0),
			m_collisionFilterGroup(btBroadphaseProxy::DefaultFilter),
			m_collisionFilterMask(btBroadphaseProxy::AllFilter),
			//@BP Mod
			m_flags(0)
		{
		}

		virtual bool needsCollision(btBroadphaseProxy* proxy0) const
		{
			bool collides = (proxy0->m_collisionFilterGroup & m_collisionFilterMask) != 0;
			collides = collides && (m_collisionFilterGroup & proxy0->m_collisionFilterMask);
			return collides;
		}


		virtual	btScalar	addSingleResult(LocalRayResult& rayResult,bool normalInWorldSpace) = 0;
	};

	struct	LocalRayResult
	{
		LocalRayResult(const btCollisionObject*	collisionObject, 
			LocalShapeInfo*	localShapeInfo,
			const btVector3&		hitNormalLocal,
			btScalar hitFraction)
		:m_collisionObject(collisionObject),
		m_localShapeInfo(localShapeInfo),
		m_hitNormalLocal(hitNormalLocal),
		m_hitFraction(hitFraction)
		{
		}

		const btCollisionObject*		m_collisionObject;
		LocalShapeInfo*			m_localShapeInfo;
		btVector3				m_hitNormalLocal;
		btScalar				m_hitFraction;

	};

	///LocalShapeInfo gives extra information for complex shapes
	///Currently, only btTriangleMeshShape is available, so it just contains triangleIndex and subpart
	struct	LocalShapeInfo
	{
		int	m_shapePart;
		int	m_triangleIndex;
		
		//const btCollisionShape*	m_shapeTemp;
		//const btTransform*	m_shapeLocalTransform;
	};

	%nestedworkaround btSoftBody::eAeroModel;
	%nestedworkaround btSoftBody::eVSolver; 
	%nestedworkaround btSoftBody::ePSolver; 
	%nestedworkaround btSoftBody::Element;
	%nestedworkaround btSoftBody::Material;
	%nestedworkaround btSoftBody::Feature;
	%nestedworkaround btSoftBody::Node;
	%nestedworkaround btSoftBody::Link;
	%nestedworkaround btSoftBody::Face;
	%nestedworkaround btSoftBody::Config;
	%nestedworkaround btSoftBody::fCollision;

	%nestedworkaround btSoftBody::Cluster;
	%nestedworkaround btSoftBody::Impulse;
	%nestedworkaround btSoftBody::Body;
	%nestedworkaround btSoftBody::SBJoint;
	%nestedworkaround btSoftBody::LJoint;
	%nestedworkaround btSoftBody::AJoint;
	%nestedworkaround btSoftBody::CJoint;
 

	%nestedworkaround btCollisionWorld::RayResultCallback;
	%nestedworkaround btCollisionWorld::LocalRayResult;
	%nestedworkaround btCollisionWorld::LocalShapeInfo;

%{
    #include "btSoftBody.h"
	#include "btSoftBodyData.h"
	#include "btSoftBodyConcaveCollisionAlgorithm.h"
	#include "btSoftBodyHelpers.h"
	#include "btSoftBodyRigidBodyCollisionConfiguration.h"
	#include "btSoftRigidCollisionAlgorithm.h"
	#include "btSoftRigidDynamicsWorld.h"
	#include "btSoftSoftCollisionAlgorithm.h"
	#include "btSparseSDF.h"

	#include "btSoftBodySolvers.h"
	#include "btDefaultSoftBodySolver.h"

	#include "btSoftBodySolverVertexBuffer.h"
%}

    %include "arrays_csharp.i"

  

    #define _WIN32
	%import "../LinearMath/btScalar.h"
	%import "../LinearMath/btAlignedObjectArray.h"
	%import  "../btBulletCollisionCommon.h"
	%import  "../btBulletDynamicsCommon.h"
    %include "btSoftBody.h"
	%include "btSoftBodyData.h"
	%include "btSoftBodyConcaveCollisionAlgorithm.h"

	%apply float INPUT[]  { const btScalar*	vertices }
	%apply int INPUT[]  { const int* triangles }
	%include "btSoftBodyHelpers.h"

	%clear const btScalar*	vertices;
    %clear const int* triangles;

	%include "btSoftBodyRigidBodyCollisionConfiguration.h"
	%include "btSoftRigidCollisionAlgorithm.h"
	%include "btSoftRigidDynamicsWorld.h"
	%include "btSoftSoftCollisionAlgorithm.h"
	%include "btSparseSDF.h"

	%include "btSoftBodySolvers.h"
	%include "btDefaultSoftBodySolver.h"

	%include "btSoftBodySolverVertexBuffer.h"

	
// template inst
%template(btSparseSdf3) btSparseSdf<3>;
%template(btAlignedObjectArrayInt) btAlignedObjectArray<int>;
%template(btAlignedObjectArrayeVSolver) btAlignedObjectArray<eVSolver::_>;
%template(btAlignedObjectArrayePSolver) btAlignedObjectArray<ePSolver::_>;

%template(btAlignedObjectArrayNode) btAlignedObjectArray<Node>;
%template(btAlignedObjectArrayLink) btAlignedObjectArray<Link>;
%template(btAlignedObjectArrayFace) btAlignedObjectArray<Face>;
%template(btAlignedObjectArrayMaterialPtr) btAlignedObjectArray<Material*>;
%template(btAlignedObjectArrayVector3) btAlignedObjectArray<btVector3>;
%template(btAlignedObjectArraySBJointPtr) btAlignedObjectArray<SBJoint*>;


%{
typedef btSoftBody::eAeroModel eAeroModel;
typedef btSoftBody::eVSolver eVSolver; 
typedef btSoftBody::ePSolver ePSolver; 
typedef btSoftBody::Element Element;
typedef btSoftBody::Material Material;
typedef btSoftBody::Feature Feature;
typedef btSoftBody::Node Node;
typedef btSoftBody::Link Link;
typedef btSoftBody::Face Face;
typedef btSoftBody::Config Config;
typedef btAlignedObjectArray<eVSolver::_>	tVSolverArray;
typedef btAlignedObjectArray<ePSolver::_>	tPSolverArray;
typedef btSoftBody::fCollision fCollision;

typedef btSoftBody::Cluster Cluster;
typedef btSoftBody::Impulse Impulse;
typedef btSoftBody::Body Body;
typedef btSoftBody::SBJoint SBJoint;
typedef btSoftBody::LJoint LJoint;
typedef btSoftBody::AJoint AJoint;
typedef btSoftBody::CJoint CJoint;

 
typedef btCollisionWorld::RayResultCallback RayResultCallback;
typedef btCollisionWorld::LocalRayResult LocalRayResult;
typedef btCollisionWorld::LocalShapeInfo LocalShapeInfo;

typedef btAlignedObjectArray<btScalar>	tScalarArray;
typedef btAlignedObjectArray<btVector3>	tVector3Array;
%}

// for later compiler pass.....
// btDbvt *temp = new btDbvt();
//  result.clone(*temp);
//  jresult = temp;

// command for swig
// ..\..\swig\swig -namespace BulletCSharp -c++ -csharp -outdir wrap BulletSoftBodySwig.i