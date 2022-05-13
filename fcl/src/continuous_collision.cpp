#include "fcl/collision.h"
#include "fcl/continuous_collision.h"
#include "fcl/ccd/motion.h"
#include "fcl/BVH/BVH_model.h"
#include "fcl/traversal/traversal_node_bvhs.h"
#include "fcl/traversal/traversal_node_setup.h"
#include "fcl/collision_node.h"
#include "fcl/ccd/conservative_advancement.h"
#include <iostream>
#include <math.h>

// lerp include
#include "fcl/tm/ModelParameters.h"
#include "fcl/tm/genetic_algorithm.h"
#include "fcl/tm/Taylor_Function.h"
// lerp

namespace fcl
{

template<typename GJKSolver>
ConservativeAdvancementFunctionMatrix<GJKSolver>& getConservativeAdvancementFunctionLookTable()
{
  static ConservativeAdvancementFunctionMatrix<GJKSolver> table;
  return table;
}

MotionBasePtr getMotionBase(const Transform3f& tf_beg, const Transform3f& tf_end, CCDMotionType motion_type)
{
  switch(motion_type)
  {
  case CCDM_TRANS:
    return MotionBasePtr(new TranslationMotion(tf_beg, tf_end));
    break;
  case CCDM_LINEAR:
    return MotionBasePtr(new InterpMotion(tf_beg, tf_end));
    break;
  case CCDM_SCREW:
    return MotionBasePtr(new ScrewMotion(tf_beg, tf_end));
    break;
  case CCDM_SPLINE:
    return MotionBasePtr(new SplineMotion(tf_beg, tf_end));
    break;
  case CCDM_LERP:
  {
      // modbyme
//      ROS_INFO("Motion and continuousCollide is not completed yet.");
//      return MotionBasePtr(new LerpMotion(joint_beg, joint_end));   // add case CCDM_LERP
      return MotionBasePtr ();
      break;
  }
  default:
    return MotionBasePtr();
  }
}

FCL_REAL continuousCollideNaive(const CollisionGeometry* o1, const MotionBase* motion1,
                                const CollisionGeometry* o2, const MotionBase* motion2,
                                const ContinuousCollisionRequest& request,
                                ContinuousCollisionResult& result)
{
  std::size_t n_iter = std::min(request.num_max_iterations, (std::size_t)ceil(1 / request.toc_err));
  Transform3f cur_tf1, cur_tf2;
  for(std::size_t i = 0; i < n_iter; ++i)
  {
    FCL_REAL t = i / (FCL_REAL) (n_iter - 1);
    motion1->integrate(t);
    motion2->integrate(t);
    
    motion1->getCurrentTransform(cur_tf1);
    motion2->getCurrentTransform(cur_tf2);

    CollisionRequest c_request;
    CollisionResult c_result;

    if(collide(o1, cur_tf1, o2, cur_tf2, c_request, c_result))
    {
      result.is_collide = true;
      result.time_of_contact = t;
      result.contact_tf1 = cur_tf1;
      result.contact_tf2 = cur_tf2;
      return t;
    }
  }

  result.is_collide = false;
  result.time_of_contact = FCL_REAL(1);
  return result.time_of_contact;
}

namespace details
{
template<typename BV>
FCL_REAL continuousCollideBVHPolynomial(const CollisionGeometry* o1_, const TranslationMotion* motion1,
                                        const CollisionGeometry* o2_, const TranslationMotion* motion2,
                                        const ContinuousCollisionRequest& request,
                                        ContinuousCollisionResult& result)
{
  const BVHModel<BV>* o1__ = static_cast<const BVHModel<BV>*>(o1_);
  const BVHModel<BV>* o2__ = static_cast<const BVHModel<BV>*>(o2_);

  // ugly, but lets do it now.
  BVHModel<BV>* o1 = const_cast<BVHModel<BV>*>(o1__);
  BVHModel<BV>* o2 = const_cast<BVHModel<BV>*>(o2__);
  std::vector<Vec3f> new_v1(o1->num_vertices);
  std::vector<Vec3f> new_v2(o2->num_vertices);

  for(std::size_t i = 0; i < new_v1.size(); ++i)
    new_v1[i] = o1->vertices[i] + motion1->getVelocity();
  for(std::size_t i = 0; i < new_v2.size(); ++i)
    new_v2[i] = o2->vertices[i] + motion2->getVelocity();

  o1->beginUpdateModel();
  o1->updateSubModel(new_v1);
  o1->endUpdateModel(true, true);

  o2->beginUpdateModel();
  o2->updateSubModel(new_v2);
  o2->endUpdateModel(true, true);

  MeshContinuousCollisionTraversalNode<BV> node;
  CollisionRequest c_request;

  motion1->integrate(0);
  motion2->integrate(0);
  Transform3f tf1, tf2;
  motion1->getCurrentTransform(tf1);
  motion2->getCurrentTransform(tf2);
  if(!initialize<BV>(node, *o1, tf1, *o2, tf2, c_request))
    return -1.0;

  collide(&node);

  result.is_collide = (node.pairs.size() > 0);
  result.time_of_contact = node.time_of_contact;

  if(result.is_collide)
  {
    motion1->integrate(node.time_of_contact);
    motion2->integrate(node.time_of_contact);
    motion1->getCurrentTransform(tf1);
    motion2->getCurrentTransform(tf2);
    result.contact_tf1 = tf1;
    result.contact_tf2 = tf2;
  }

  return result.time_of_contact;
}

}

FCL_REAL continuousCollideBVHPolynomial(const CollisionGeometry* o1, const TranslationMotion* motion1,
                                        const CollisionGeometry* o2, const TranslationMotion* motion2,
                                        const ContinuousCollisionRequest& request,
                                        ContinuousCollisionResult& result)
{
  switch(o1->getNodeType())
  {
  case BV_AABB:
    if(o2->getNodeType() == BV_AABB)
      return details::continuousCollideBVHPolynomial<AABB>(o1, motion1, o2, motion2, request, result);
    break;
  case BV_OBB:
    if(o2->getNodeType() == BV_OBB)
      return details::continuousCollideBVHPolynomial<OBB>(o1, motion1, o2, motion2, request, result);
    break;
  case BV_RSS:
    if(o2->getNodeType() == BV_RSS)
      return details::continuousCollideBVHPolynomial<RSS>(o1, motion1, o2, motion2, request, result);
    break;
  case BV_kIOS:
    if(o2->getNodeType() == BV_kIOS)
      return details::continuousCollideBVHPolynomial<kIOS>(o1, motion1, o2, motion2, request, result);
    break;
  case BV_OBBRSS:
    if(o2->getNodeType() == BV_OBBRSS)
      return details::continuousCollideBVHPolynomial<OBBRSS>(o1, motion1, o2, motion2, request, result);
    break;
  case BV_KDOP16:
    if(o2->getNodeType() == BV_KDOP16)
      return details::continuousCollideBVHPolynomial<KDOP<16> >(o1, motion1, o2, motion2, request, result);
    break;
  case BV_KDOP18:
    if(o2->getNodeType() == BV_KDOP18)
      return details::continuousCollideBVHPolynomial<KDOP<18> >(o1, motion1, o2, motion2, request, result);
    break;
  case BV_KDOP24:
    if(o2->getNodeType() == BV_KDOP24)
      return details::continuousCollideBVHPolynomial<KDOP<24> >(o1, motion1, o2, motion2, request, result);
    break;
  default:
    ;
  }

  std::cerr << "Warning: BV type not supported by polynomial solver CCD" << std::endl;

  return -1;
}

namespace details
{

template<typename NarrowPhaseSolver>
FCL_REAL continuousCollideConservativeAdvancement(const CollisionGeometry* o1, const MotionBase* motion1,
                                                  const CollisionGeometry* o2, const MotionBase* motion2,
                                                  const NarrowPhaseSolver* nsolver_,
                                                  const ContinuousCollisionRequest& request,
                                                  ContinuousCollisionResult& result)
{
  const NarrowPhaseSolver* nsolver = nsolver_;
  if(!nsolver_)
    nsolver = new NarrowPhaseSolver();

  const ConservativeAdvancementFunctionMatrix<NarrowPhaseSolver>& looktable = getConservativeAdvancementFunctionLookTable<NarrowPhaseSolver>();

  NODE_TYPE node_type1 = o1->getNodeType();
  NODE_TYPE node_type2 = o2->getNodeType();

  FCL_REAL res = -1;
  
  if(!looktable.conservative_advancement_matrix[node_type1][node_type2])
  {
    std::cerr << "Warning: collision function between node type " << node_type1 << " and node type " << node_type2 << " is not supported"<< std::endl;
  }
  else
  {
    res = looktable.conservative_advancement_matrix[node_type1][node_type2](o1, motion1, o2, motion2, nsolver, request, result);
  }

  if(!nsolver_)
    delete nsolver;

  if(result.is_collide)
  {
    motion1->integrate(result.time_of_contact);
    motion2->integrate(result.time_of_contact);

    Transform3f tf1, tf2;
    motion1->getCurrentTransform(tf1);
    motion2->getCurrentTransform(tf2);
    result.contact_tf1 = tf1;
    result.contact_tf2 = tf2;
  }

  return res;
}

}


FCL_REAL continuousCollideConservativeAdvancement(const CollisionGeometry* o1, const MotionBase* motion1,
                                                  const CollisionGeometry* o2, const MotionBase* motion2,
                                                  const ContinuousCollisionRequest& request,
                                                  ContinuousCollisionResult& result)
{
  switch(request.gjk_solver_type)
  {
  case GST_LIBCCD:
    {
      GJKSolver_libccd solver;
      return details::continuousCollideConservativeAdvancement(o1, motion1, o2, motion2, &solver, request, result);
    }
  case GST_INDEP:
    {
      GJKSolver_indep solver;
      return details::continuousCollideConservativeAdvancement(o1, motion1, o2, motion2, &solver, request, result);
    }
  default:
    return -1;
  }
}

FCL_REAL continuousCollideLerp(const CollisionGeometry* o1, const MotionBase* motion1,
                               const CollisionGeometry* o2, const MotionBase* motion2,
                               const ContinuousCollisionRequest& request,
                               ContinuousCollisionResult& result)
{
    // Modbyme continuousCollideLerp Algorithm completed in here.
    // change MotionBase* to LerpMotion*
    // return continuousCollideLerp(o1, motion1, o2, motion2, request, result)
    return -1;
}

FCL_REAL continuousCollideLerp(const CollisionGeometry* o1, const LerpMotion* motion1,
                               const CollisionGeometry* o2, const LerpMotion* motion2,
                               const ContinuousCollisionRequest& request,
                               ContinuousCollisionResult& result)
{
    // Modbyme continuousCollideLerp Algorithm completed in here.
    // initial position and posture, read the data from the txt file or moveit
    // (add data type frame)
    // init_frame_hm();
    ReadUrdfFile();

    // set the global time interval
    // the time interval should be got by the time interval the moveit frame
    cInterval T(0,0.25);
    setGlobalTimeInterval(T.i[0],T.i[1]);
    double t = T.i[1]-T.i[0];

    int joint_num = 7;
    // init and setup the yaw
    init_yaw(t);
    setupyaw(t, motion1, motion2, joint_num);

    // calculate the rotation matrix Rinit from the above

    // cTM3Matrix33 Rinit[2][10];

    // calculate the rotation matrix Ryaw from the LerpMotion->getJoint*
    // cTM3Matrix33 Ryaw[2][10];

    // calculate the transform matrix HM from Rinit and Ryaw
    // cTM3Matrix44 HM[2][10];
    setupHM(joint_num);
//    showHM(joint_num);

    // Loop: Start

    fcl::FCL_REAL dis = 0;
    __T = T;
    __toc_err = request.toc_err;


    if(arm_flag)
    {
        // single-arm

        IDtypedecoding(request.obj1_ID, link_type[0], __linki);
        IDtypedecoding(request.obj2_ID, link_type[1], __linkj);
//        BVHModel<OBBRSS> obj1;
//        BVHModel<OBBRSS> obj2;
        BVHModel<SPHERE> obj1;
        BVHModel<SPHERE> obj2;

        if (link_type[0] == 0) {
            std::string filenames;
            filenames = "link" + std::to_string(__linki) + ".obj";
            model_load(filenames, obj1);
        } else if (link_type[0] == 1) {
            std::string filenames = "hand.obj";
            model_load(filenames, obj1);
        } else if (link_type[0] == 2 || link_type[0] == 3) {
            std::string filenames = "finger.obj";
            model_load(filenames, obj1);
        } else {}

        if (link_type[1] == 0) {
            std::string filenames;
            filenames = "link" + std::to_string(__linkj) + ".obj";
            model_load(filenames, obj2);
        } else if (link_type[1] == 1) {
            std::string filenames = "hand.obj";
            model_load(filenames, obj2);
        } else if (link_type[1] == 2 || link_type[1] == 3) {
            std::string filenames = "finger.obj";
            model_load(filenames, obj2);
        } else {}

        clock_t start = clock();
        tra_num = 0;
        dis = BVHTraversal(0, 0, obj1, obj2);
        clock_t end = clock();
        std::cout << "detection running time: " << double(end-start) / CLOCKS_PER_SEC << "s \n";
        std::cout << "The number of times the function was called recursively: " << tra_num << "\n";
        std::cout << "the num of bv pairs is: " << obj1.num_tris << " * " << obj2.num_tris
                  << " = " << obj1.num_tris * obj2.num_tris << "\n";
    }
    else
    {
        // dual-arm

    }
    std::cout << "dis: " << dis << "\n\n";
    return dis;
    // Loop: END
}
  
FCL_REAL continuousCollide(const CollisionGeometry* o1, const MotionBase* motion1,
                           const CollisionGeometry* o2, const MotionBase* motion2,
                           const ContinuousCollisionRequest& request,
                           ContinuousCollisionResult& result)
{
  switch(request.ccd_solver_type)
  {
  case CCDC_NAIVE:
      return continuousCollideNaive(o1, motion1,
                                    o2, motion2,
                                    request,
                                    result);
      break;
  case CCDC_CONSERVATIVE_ADVANCEMENT:
    return continuousCollideConservativeAdvancement(o1, motion1,
                                                    o2, motion2,
                                                    request,
                                                    result);
    break;
  case CCDC_RAY_SHOOTING:
    if(o1->getObjectType() == OT_GEOM && o2->getObjectType() == OT_GEOM && request.ccd_motion_type == CCDM_TRANS)
    {
      
    }
    else
      std::cerr << "Warning! Invalid continuous collision setting" << std::endl;
    break;
  case CCDC_POLYNOMIAL_SOLVER:
    if(o1->getObjectType() == OT_BVH && o2->getObjectType() == OT_BVH && request.ccd_motion_type == CCDM_TRANS)
    {
      return continuousCollideBVHPolynomial(o1, (const TranslationMotion*)motion1,
                                            o2, (const TranslationMotion*)motion2,
                                            request, result);
    }
    else
      std::cerr << "Warning! Invalid continuous collision checking" << std::endl;
    break;
  case CCDC_LERP:
      return continuousCollideLerp(o1, motion1,
                                    o2, motion2,
                                    request,
                                    result);
      break;
  default:
    std::cerr << "Warning! Invalid continuous collision setting" << std::endl;
  }

  return -1;
}

FCL_REAL continuousCollide(const CollisionGeometry* o1, const Transform3f& tf1_beg, const Transform3f& tf1_end,
                           const CollisionGeometry* o2, const Transform3f& tf2_beg, const Transform3f& tf2_end,
                           const ContinuousCollisionRequest& request,
                           ContinuousCollisionResult& result)
{
  MotionBasePtr motion1 = getMotionBase(tf1_beg, tf1_end, request.ccd_motion_type);
  MotionBasePtr motion2 = getMotionBase(tf2_beg, tf2_end, request.ccd_motion_type);

  return continuousCollide(o1, motion1.get(), o2, motion2.get(), request, result);
}


FCL_REAL continuousCollide(const CollisionObject* o1, const Transform3f& tf1_end,
                           const CollisionObject* o2, const Transform3f& tf2_end,
                           const ContinuousCollisionRequest& request,
                           ContinuousCollisionResult& result)
{
  return continuousCollide(o1->collisionGeometry().get(), o1->getTransform(), tf1_end,
                           o2->collisionGeometry().get(), o2->getTransform(), tf2_end,
                           request, result);
}

FCL_REAL continuousCollide(const CollisionGeometry* o1, const std::vector<double> state1_beg, const std::vector<double> state1_end,
                           const CollisionGeometry* o2, const std::vector<double> state2_beg, const std::vector<double> state2_end,
                           const ContinuousCollisionRequest& request,
                           ContinuousCollisionResult& result)  // Modbyme
{
    // transform vector<double> to vector<float>
    std::vector<float> state1_beg_(state1_beg.begin(), state1_beg.end());
    std::vector<float> state1_end_(state1_end.begin(), state1_end.end());
    std::vector<float> state2_beg_(state2_beg.begin(), state2_beg.end());
    std::vector<float> state2_end_(state2_end.begin(), state2_end.end());

//    MotionBasePtr motion1 = MotionBasePtr (new LerpMotion(state1_beg_, state1_end_));
//    MotionBasePtr motion2 = MotionBasePtr (new LerpMotion(state2_beg_, state2_end_));
//    return continuousCollide(o1, motion1.get(), o2, motion2.get(), request, result);
    LerpMotion motion1(state1_beg_,state1_end_);
    LerpMotion motion2(state2_beg_,state2_end_);
    return continuousCollideLerp(o1, &motion1, o2, &motion2, request, result);
}

FCL_REAL continuousCollideNaive(const CollisionGeometry* o1, const MotionAM* motion1,
                                const CollisionGeometry* o2, const MotionAM* motion2,
                                const ContinuousCollisionRequest& request,
                                ContinuousCollisionResult& result)
{
    std::size_t n_iter = std::min(request.num_max_iterations, (std::size_t)ceil(1 / request.toc_err));
    Transform3f cur_tf1, cur_tf2;
    for(std::size_t i = 0; i < n_iter; ++i)
    {


        FCL_REAL t = i / (FCL_REAL) (n_iter - 1);
        motion1->integrate(t);
        motion2->integrate(t);

        motion1->getCurrentTransform(cur_tf1);
        motion2->getCurrentTransform(cur_tf2);

        CollisionRequest c_request;
        CollisionResult c_result;

        if(collide(o1, cur_tf1, o2, cur_tf2, c_request, c_result))
        {
            result.is_collide = true;
            result.time_of_contact = t;
            result.contact_tf1 = cur_tf1;
            result.contact_tf2 = cur_tf2;
            return t;
        }
    }

    result.is_collide = false;
    result.time_of_contact = FCL_REAL(1);
    return result.time_of_contact;

}

FCL_REAL continuousCollideConservativeAdvancement(const CollisionGeometry* o1, const MotionAM* motion1,
                                                  const CollisionGeometry* o2, const MotionAM* motion2,
                                                  const ContinuousCollisionRequest& request,
                                                  ContinuousCollisionResult& result)    // Modbyme
{
    switch(request.gjk_solver_type)
    {
        case GST_LIBCCD:
        {
            GJKSolver_libccd solver;
            return continuousCollideConservativeAdvancement(o1, motion1, o2, motion2, &solver, request, result);
        }
        case GST_INDEP:
        {
            GJKSolver_indep solver;
            return continuousCollideConservativeAdvancement(o1, motion1, o2, motion2, &solver, request, result);
        }
        default:
            return -1;
    }
}

template<typename NarrowPhaseSolver>
FCL_REAL continuousCollideConservativeAdvancement(const CollisionGeometry* o1, const MotionAM* motion1,
                                                  const CollisionGeometry* o2, const MotionAM* motion2,
                                                  const NarrowPhaseSolver* nsolver_,
                                                  const ContinuousCollisionRequest& request,
                                                  ContinuousCollisionResult& result)   // Modbyme
{
    const NarrowPhaseSolver* nsolver = nsolver_;
    if(!nsolver_)
        nsolver = new NarrowPhaseSolver();

//    const ConservativeAdvancementFunctionMatrix<NarrowPhaseSolver>& looktable = getConservativeAdvancementFunctionLookTable<NarrowPhaseSolver>();
//
//    NODE_TYPE node_type1 = o1->getNodeType();
//    NODE_TYPE node_type2 = o2->getNodeType();

    FCL_REAL res = -1;

//    if(!looktable.conservative_advancement_matrix[node_type1][node_type2])
//    {
//        std::cerr << "Warning: collision function between node type " << node_type1 << " and node type " << node_type2 << " is not supported"<< std::endl;
//    }
//    else
//    {
//        res = looktable.conservative_advancement_matrix[node_type1][node_type2](o1, motion1, o2, motion2, nsolver, request, result);
//    }

    res = BVHConservativeAdvancement<OBBRSS, NarrowPhaseSolver>(o1, motion1, o2, motion2, nsolver, request, result);

    if(!nsolver_)
        delete nsolver;

    if(result.is_collide)
    {
        motion1->integrate(result.time_of_contact);
        motion2->integrate(result.time_of_contact);

        Transform3f tf1, tf2;
        motion1->getCurrentTransform(tf1);
        motion2->getCurrentTransform(tf2);
        result.contact_tf1 = tf1;
        result.contact_tf2 = tf2;
    }

    return res;
}

// Modbyme
template<typename BV, typename NarrowPhaseSolver>
FCL_REAL BVHConservativeAdvancement(const CollisionGeometry* o1, const MotionAM* motion1, const CollisionGeometry* o2, const MotionAM* motion2, const NarrowPhaseSolver* nsolver, const ContinuousCollisionRequest& request, ContinuousCollisionResult& result)
{
    const BVHModel<BV>* obj1 = static_cast<const BVHModel<BV>*>(o1);
    const BVHModel<BV>* obj2 = static_cast<const BVHModel<BV>*>(o2);

    CollisionRequest c_request;
    CollisionResult c_result;
    FCL_REAL toc = request.toc_err;
    bool is_collide = conservativeAdvancement(*obj1, motion1, *obj2, motion2, c_request, c_result, toc);

    result.is_collide = is_collide;
    result.time_of_contact = toc;

    return toc;
}

template<typename BV, typename ConservativeAdvancementOrientedNode>
bool conservativeAdvancementMeshOriented(const BVHModel<BV>& o1, const MotionAM* motion1,
                                         const BVHModel<BV>& o2, const MotionAM* motion2,
                                         const CollisionRequest& request, CollisionResult& result, FCL_REAL& toc) //Modbyme
{
    Transform3f tf1, tf2;
    motion1->getCurrentTransform(tf1);
    motion2->getCurrentTransform(tf2);
    motion1->computetf0();
    motion1->computetf1();
    motion2->computetf0();
    motion2->computetf1();
    Transform3f tf10, tf11, tf20, tf21;
    motion1->getTF0(tf10);
    motion1->getTF1(tf11);
    motion2->getTF0(tf20);
    motion2->getTF1(tf21);

//    const MotionBasePtr* motion1_base = new MotionBasePtr (getMotionBase(tf1, tf1, CCDM_TRANS));
//    const MotionBasePtr* motion2_base = new MotionBasePtr (getMotionBase(tf2, tf2, CCDM_TRANS));
    const TranslationMotion *motion1_base = new TranslationMotion (tf10,tf11);
    const TranslationMotion *motion2_base = new TranslationMotion (tf20,tf21);

    // whether the first start configuration is in collision
    if(collide(&o1, tf1, &o2, tf2, request, result))
    {
        toc = 0;
        return true;
    }

    ConservativeAdvancementOrientedNode node;

    initialize(node, o1, tf1, o2, tf2);
    node.t_err = toc;

    node.motion1 = motion1_base;
    node.motion2 = motion2_base;

    do
    {
        node.motion1->getCurrentTransform(tf1);
        node.motion2->getCurrentTransform(tf2);

        // compute the transformation from 1 to 2
        Transform3f tf;
        relativeTransform(tf1, tf2, tf);
        node.R = tf.getRotation();
        node.T = tf.getTranslation();

        node.delta_t = 1;
        node.min_distance = std::numeric_limits<FCL_REAL>::max();

        distanceRecurse(&node, 0, 0, NULL);

        if(node.delta_t <= node.t_err)
        {
            // std::cout << node.delta_t << " " << node.t_err << std::endl;
            break;
        }

        node.toc += node.delta_t;
        if(node.toc > 1)
        {
            node.toc = 1;
            break;
        }

        motion1->integrate(node.toc);
        motion2->integrate(node.toc);

        Transform3f tf1_tmp, tf2_tmp;
        motion1->getCurrentTransform(tf1_tmp);
        motion2->getCurrentTransform(tf2_tmp);
        node.motion1->assignTransform(tf1_tmp);
        node.motion2->assignTransform(tf1_tmp);
    }
    while(1);

    toc = node.toc;
    std::cout << node.min_distance;

    if(node.toc < 1)
        return true;

    return false;
}

bool conservativeAdvancement(const BVHModel<OBBRSS>& o1, const MotionAM* motion1,
                             const BVHModel<OBBRSS>& o2, const MotionAM* motion2,
                             const CollisionRequest& request, CollisionResult& result, FCL_REAL& toc) // Modbyme
{
    return conservativeAdvancementMeshOriented<OBBRSS, MeshConservativeAdvancementTraversalNodeOBBRSS>(o1, motion1, o2, motion2, request, result, toc);
}

FCL_REAL collide(const ContinuousCollisionObject* o1, const ContinuousCollisionObject* o2,
                 const ContinuousCollisionRequest& request,
                 ContinuousCollisionResult& result)
{
  return continuousCollide(o1->collisionGeometry().get(), o1->getMotion(),
                           o2->collisionGeometry().get(), o2->getMotion(),
                           request, result);
}

}
