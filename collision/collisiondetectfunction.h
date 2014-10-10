///
///@brief		�v���~�e�B�u�Փˊ֘A�֐��Q
/// 					- 2D�Փ˔���
/// 						- �_(Pt)
/// 						- �~(Circle)
/// 						- ��(line)
/// 						- ����(seg)
/// 						- ������/���C(Ray)
/// 						- Axis Aligned Bounding Box(Aabb)
/// 					- 3D�Փ˔���
/// 						- �_(Pt)
/// 						- ��(Sphere)
/// 						- ��(line)
/// 						- ����(seg)
/// 						- ������/���C(Ray)
/// 						- ����(Plane)
/// 						- �O�p�`(Tri)
/// 						- �l�p�`(Square)
/// 						- Axis Aligned Bounding Box(Aabb)
/// 						- Oriented Bounding Box(Obb)
///
///@attention	DirectX�p�ɍ�����W�n�ł���
///@auther		RockField
///@date		2010/04/29
///@file		collisiondetection.h
///

#ifndef _RF_COLLISION_DITECT_FUNCTION_H_
#define _RF_COLLISION_DITECT_FUNCTION_H_

#include "../math/index.h"
#include "boundingbox.h"
#include "plane.h"
#include "frustum.h"

///@brief rf
namespace rf
{
	///@brief collision
	namespace collision
	{
		///@name	definitions
		///@brief	�����s��A��x�N�g�����̒�`
		// @{

		/////@brief	�O���[�o���֐�
		// namespace util
		// {
			///@defgroup	intersection
			///@brief	�����A�Փ˔���
			// @{

			///@brief	�_-�_�̋��������߂�
			///@param	a_	�_A�̍��W
			///@param   b_	�_B�̍��W
			///@retval  �_-�_�̋���
			float PtPtDistance(
				const math::t_vector2&	a_,
				const math::t_vector2&	b_);

			///@brief	�~-�~�̏Փ˂𔻒肷��
			///@param	centerA_	�~A�̒��S���W
			///@param	radiusA_	�~A�̔��a
			///@param   centerB_	�~B�̒��S���W
			///@param   radiusB_	�~B�̔��a
			///@retval	true		�Փ˂���
			///@retval	false		�Փ˂Ȃ�
			///@note	�~�̔��a�𑫂��ĉ~�Ɠ_�̏Փ˔���Ɏˉe����
			float CircleCircleCol(
				const math::t_vector2&	centerA_,
				float 					radiusA_,
				const math::t_vector2&	centerB_,
				float 					radiusB_);

			#if 0
				///@brief	����-�����̍ŋߐړ_
				///@param	segAP_		��A�̎n�_
				///@param	segADir_	��A�̏I�_
				///@param segBP_		��B�̎n�_
				///@param segBDir_	��B�̏I�_
				///@param	proximateA_	[out]��A��̍ŋߐړ_
				///@param	proximateB_	[out]��B��̍ŋߐړ_
				void SegSegCol(
					const math::t_vector2&	segAP_,
					const math::t_vector2&	segAQ_,
					const math::t_vector2&	segBP_,
					const math::t_vector2&	segBQ_,
					math::t_vector2*		intersection_);
			#endif

			///@brief	AABB-�_�̏Փ˂𔻒肷��
			///@param	aabb_		AABB
			///@param	p_			�_�̍��W
			///@retval	true		�Փ˂���
			///@retval	false		�Փ˂Ȃ�
			bool AabbPtCol(
				const CAabb2&			aabb_,
				const math::t_vector2&	p_);

			///@brief	AABB-�_�ŋߐړ_���擾����
			///@param	aabb_			AABB
			///@param	p_				�_�̍��W
			///@param	proximate_		[out]�ŋߐړ_
			void AabbPtProximate(
				const CAabb2&			aabb_,
				const math::t_vector2&	p_,
				math::t_vector2*		proximate_);

			///@brief	AABB-AABB�̏Փ˂𔻒肷��
			///@param	aabbA_	AABB_A
			///@param	aabbB_	AABB_B
			///@retval	true		�Փ˂���
			///@retval	false		�Փ˂Ȃ�
			bool AabbAabbCol(
				const CAabb2&	aabbA_,
				const CAabb2&	aabbB_);

			// @}
		// }
	}

	///@brief	3D�Փ˔���
	namespace collision
	{
		typedef enum {
			RF_COLLIDE_NONE   = 0,
			RF_COLLIDE_VERTEX,	// ���_�ƕ��ʂ̏Փ�
			RF_COLLIDE_EDGE,	// �ӂƕӂ̏Փ�
			RF_COLLIDE_NORMAL,	// ���̑��̏Փ�
		}e_collide_bits;

		/////@brief	�O���[�o���֐�
		// namespace util
		// {
			///@defgroup	intersection
			///@brief	�����A�Փ˔���
			// @{

			///@brief	��]�̂̈ʒu�Ƒ��x�Ɗp���x����A����_�̑��x�����߂�
			///@param	world_		��]�̂̃��[���h�ϊ��s��
			///@param   v_			��]�̂̑��x
			///@param	w_			��]�̂̊p���x
			///@param	p_			���x�����߂����_�̉�]�̃��[�J�����W
			///@param   sweepVec_	[out]���[���h�ł̓_�̑��x
			void RotSweep(
				const math::t_matrix4&	world_,
				const math::t_vector3&	v_,
				const math::t_vector3&	w_,
				const math::t_vector3&	p_,
				math::t_vector3*		sweepVec_);

			///@brief	�_-�_�̋��������߂�
			///@param	a_	�_A�̍��W
			///@param   b_	�_B�̍��W
			///@retval  �_-�_�̋���
			float PtPtDistance(
				const math::t_vector3&	a_,
				const math::t_vector3&	b_);

			///@brief	�~-�~�̏Փ˂𔻒肷��
			///@param	centerA_	�~A�̒��S���W
			///@param	radiusA_	�~A�̔��a
			///@param   centerB_	�~B�̒��S���W
			///@param   radiusB_	�~B�̔��a
			///@retval  �_-�_�̋���
			float SphereSphereCol(
				const math::t_vector3&	a_,
				float 					radiusA_,
				const math::t_vector3&	b_,
				float 					radiusB_);

			///@brief	�_-���̋��������߂�
			///@param	p_			�_�̍��W
			///@param	lineP_		����̔C�ӂ̓_
			///@param   lineDir_	���K�����ꂽ���̕���
			///@retval  �_-���̋���
			float LinePtDistance(
				const math::t_vector3&	p_,
				const math::t_vector3&	lineP_,
				const math::t_vector3&	lineDir_);

			///@brief	����ɂ���_p_�ŋߐړ_�����߂�
			///@param	p_			�_�̍��W
			///@param	lineP_		����̔C�ӂ̓_
			///@param   lineDir_	���̕���
			///@param   proximate_	�ŋߐړ_
			void LinePtProximate(
				const math::t_vector3&	p_,
				const math::t_vector3&	lineP_,
				const math::t_vector3&	lineDir_,
				math::t_vector3*		proximate_);

			///@brief	�_-�����̋��������߂�
			///@param	p_			�_�̍��W
			///@param	segP_		�����̎n�_
			///@param   segQ_		�����̏I�_
			///@retval	�_-�����̋���
			float SegPtDistance(
				const math::t_vector3&	p_,
				const math::t_vector3&	segP_,
				const math::t_vector3&	segQ_);

			///@brief	������ɂ���_p_�̍ŋߐړ_�����߂�
			///@param	p_			�_�̍��W
			///@param	segP_		�����̎n�_
			///@param   segQ_		�����̏I�_
			///@param   proximate_	[out]�ŋߐړ_
			void SegPtProximate(
				const math::t_vector3&	segP_,
				const math::t_vector3&	segQ_,
				const math::t_vector3&	p_,
				math::t_vector3*		proximate_);

			///@brief	��-���̋��������߂�
			///@param	segAP_		��A�̎n�_
			///@param	segADir_	��A�̏I�_
			///@param   segBP_		��B�̎n�_
			///@param   segBDir_	��B�̏I�_
			///@retval	��-���̋���
			float LineLineDistance(
				const math::t_vector3&	segAP_,
				const math::t_vector3&	segADir_,
				const math::t_vector3&	segBP_,
				const math::t_vector3&	segBDir_);

			///@brief	��-���̍ŋߐړ_�����߂�
			///@param	segAP_		��A�̎n�_
			///@param	segADir_	��A�̏I�_
			///@param   segBP_		��B�̎n�_
			///@param   segBDir_	��B�̏I�_
			///@param	proximateA_	[out]��A��̍ŋߐړ_
			///@param	proximateB_	[out]��B��̍ŋߐړ_
			///@note
			///
			/// 	�ŋߐړ_�����񂾐����͗����̒����ɐ����Ƃ�������
			///
			/// 		D1�EV(s, t) = 0
			/// 		D2�EV(s, t) = 0
			///
			void LineLineProximate(
				const math::t_vector3&	lineAP_,
				const math::t_vector3&	lineADir_,
				const math::t_vector3&	lineBP_,
				const math::t_vector3&	lineBDir_,
				math::t_vector3*		proximateA_,
				math::t_vector3*		proximateB_);


			///@brief	����-�����̍ŋߐړ_
			///@param	segAP_		��A�̎n�_
			///@param	segADir_	��A�̏I�_
			///@param   segBP_		��B�̎n�_
			///@param   segBDir_	��B�̏I�_
			///@param	proximateA_	[out]��A��̍ŋߐړ_
			///@param	proximateB_	[out]��B��̍ŋߐړ_
			///@note
			///
			/// 	����-�����̍ŋߐړ_
			/// 	�����Ƃ��čŋߐړ_�����߁A���ꂪ�����̊O���ł���ꍇ�́A
			/// 	�����̋ߑ��̒[�_�ƒ����Ƃ̍ŋߐړ_�����߂�B
			/// 	���ŋߐړ_���Ƃ��ɐ����̊O���ɂ���Ƃ���
			/// 	�[�_���m���ŋߐړ_
			///
			void SegSegProximate(
				const math::t_vector3&	segAP_,
				const math::t_vector3&	segAQ_,
				const math::t_vector3&	segBP_,
				const math::t_vector3&	segBQ_,
				math::t_vector3*		proximateA_,
				math::t_vector3*		proximateB_);

			///@brief	����-�_�̈ʒu�֌W�𔻒肷��
			///@param	planeP_		���ʏ�̔C�ӂ̓_
			///@param	planeN_		���ʂ̖@��
			///@param   p_			�_�̍��W
			///@retval	true		�\��
			///@retval	false		����
			bool PlanePtClassifyInside(
				const math::t_vector3&	planeP_,
				const math::t_vector3&	planeN_,
				const math::t_vector3&	p_);

			///@brief	����-�_�̈ʒu�֌W�𔻒肷��
			///@param	apex1_		���ʏ�̔C�ӂ̓_A
			///@param	apex2_		���ʏ�̔C�ӂ̓_B
			///@param	apex3_		���ʏ�̔C�ӂ̓_C
			///@param   p_			�_�̍��W
			///@retval	true		�\��
			///@retval	false		����
			///@note	���ʂ̗��\�̒�`�͍���n�ɏ�����
			bool PlanePtClassifyInside(
				const math::t_vector3&	apex1_,
				const math::t_vector3&	apex2_,
				const math::t_vector3&	apex3_,
				const math::t_vector3&	p_);

			///@brief	���ʂƓ_�̕����t���������߂�
			///@param	plane_		����
			///@param	p_			���ʂ̖@��
			///@param   depth_		[out]���ʂƓ_�̕����t����
			///@retval  ���ʂƓ_�̕����t�����A�����\��
			///@note	���ʂ̗��\�̒�`�͍���n�ɏ�����
			float PlanePtDistance(
				const CPlane&			plane_,
				const math::t_vector3&	p_);

			///@brief	���ʂƓ_�̕����t���������߂�
			///@param	planeP_		���ʏ�̔C�ӂ̓_
			///@param	planeN_		���ʂ̖@��
			///@param	p_			���ʂ̖@��
			///@param   depth_		[out]���ʂƓ_�̕����t����
			///@retval  ���ʂƓ_�̕����t�����A�����\��
			///@note	���ʂ̗��\�̒�`�͍���n�ɏ�����
			///@note	�{����P�̈ʒu��������Ȃ��Ă����������܂�̂ŏ�����
			float PlanePtDistance(
				const math::t_vector3&	planeP_,
				const math::t_vector3&	planeN_,
				const math::t_vector3&	p_);

			///@brief	���ʂƓ_�̕����t���������߂�
			///@param	apex1_		���ʏ�̔C�ӂ̓_A
			///@param	apex2_		���ʏ�̔C�ӂ̓_B
			///@param	apex3_		���ʏ�̔C�ӂ̓_C
			///@param	p_			���ʂ̖@��
			///@param   depth_		[out]���ʂƓ_�̕����t����
			///@param   normal_		[out]���ʂ̖@��
			///@retval  ���ʂƓ_�̕����t�����A�����\��
			///@note	���ʂ̗��\�̒�`�͍���n�ɏ�����
			float PlanePtDistance(
				const math::t_vector3&	apex1_,
				const math::t_vector3&	apex2_,
				const math::t_vector3&	apex3_,
				const math::t_vector3&	p_,
				float*					depth_,
				math::t_vector3*		normal_);

			///@brief	����-�����̏Փ˂𔻒肷��
			///@param	planeP_			���ʏ�̔C�ӂ̓_
			///@param	planeN_			���ʂ̖@��
			///@param	segP_			�����̎n�_
			///@param	segQ_			�����̏I�_
			///@param   intersection_	[out]�Փ˓_�̍��W�A�ȗ���
			///@retval	true			�Փ˂���
			///@retval	false			�Փ˂Ȃ�
			///@note
			///
			/// 	�@���̌��_�ƖړI�̓_�����񂾃x�N�g���Ɩ@���x�N�g���̓��ς����߂�B
			/// 	���ς����Ȃ�Ε\���A���Ȃ�Η����Ȃ̂ŁA���[�̓_�̕������قȂ�΂���
			///
			bool PlaneSegCol(
				const math::t_vector3&	planeP_,
				const math::t_vector3&	planeN_,
				const math::t_vector3&	segP_,
				const math::t_vector3&	segQ_,
				math::t_vector3*		intersection_ = NULL);

			///@brief	���ʂƔ������̏Փ˂𔻒肷��
			///@param	planeP_			���ʏ�̔C�ӂ̓_
			///@param	planeN_			���ʂ̖@��
			///@param	rayP_,			�������̎n�_
			///@param	rayDir_			�������̒����ƕ���
			///@param   intersection_	[out]�Փ˓_�̍��W�A�ȗ���
			///@retval	true			�Փ˂���
			///@retval	false			�Փ˂Ȃ�
			bool PlaneRayCol(
				const math::t_vector3&	planeP_,
				const math::t_vector3&	planeN_,
				const math::t_vector3&	rayP_,
				const math::t_vector3&	rayVec_,
				math::t_vector3*		intersection_ = NULL);

			///@brief	���ʎO�p-���̏Փˈʒu�����߂�
			///@param	apex1_			�O�p�`�̒��_A
			///@param	apex2_			�O�p�`�̒��_B
			///@param	apex3_			�O�p�`�̒��_C
			///@param	lineP_			����̔C�ӂ̓_
			///@param	lineDir_		���̕���
			///@param	normal_			[out]�O�p�`�̖@���A�ȗ��s��
			///@param   intersection_	[out]�Փ˓_�̍��W�A�ȗ���
			///@retval	true			�Փ˂���
			///@retval	false			�Փ˂Ȃ�
			///@note
			///
			/// 	�O�p�`�̗��\�̒�`�͍���n�ɏ�����
			/// 	�����ƕ��ʂ̔���̌�Astart����_�E�O�p�`��̓�_��3�̃x�N�g����
			/// 	�X�J���O�d��(�̐�)�����ׂē��������Ȃ�O�p�`����
			///
			bool TriLineCol(
				const math::t_vector3&	apex1_,
				const math::t_vector3&	apex2_,
				const math::t_vector3&	apex3_,
				const math::t_vector3&	lineP_,
				const math::t_vector3&	lineDir_,
				math::t_vector3*		normal_,
				math::t_vector3*		intersection_ = NULL);

			///@brief	�ЖʎO�p-�����̏Փˈʒu�����߂�
			///@param	apex1_			�O�p�`�̒��_A
			///@param	apex2_			�O�p�`�̒��_B
			///@param	apex3_			�O�p�`�̒��_C
			///@param	segP_			�����̎n�_
			///@param	segQ_			�����̏I�_
			///@param	normal_			[out]�O�p�`�̖@���A�ȗ��s��
			///@param   distance_		[out]�Փ˓_�̍��W�A�ȗ���
			///@param   intersection_	[out]�Փ˓_�̍��W�A�ȗ���
			///@retval	true			�Փ˂���
			///@retval	false			�Փ˂Ȃ�
			///@note
			///
			/// 	��{�I�ɂ͎O�p�`-���Ɠ���
			/// 	�O�p�`��̓_��uA+vB+wC�ŕ\�����Ƃ�
			/// 		u+v+w=1�Ȃ畽�ʏ�̓_�A�œK���̂���u=1-v-w�ŕ�������������
			/// 		v>0,w>0,v+w<1
			/// 	���ŏI�I�ȏ���
			/// 	��{�I�ɂ͎O�p�`�Ɛ��Ƃ̔���Ɠ����ɂȂ邪�A
			/// 	���̌v�Z�r���ɓ�����e�l���Ɍ��Ă������ƂŁA
			/// 	�Փ˂��Ă��Ȃ��ꍇ�𑁊��ɔ����ł���
			///
			bool TriSegCol(
				const math::t_vector3&	apex1_,
				const math::t_vector3&	apex2_,
				const math::t_vector3&	apex3_,
				const math::t_vector3&	segP_,
				const math::t_vector3&	segQ_,
				math::t_vector3*		normal_,
				float*					distance_ = NULL,
				math::t_vector3*		intersection_ = NULL);

			///@brief	���ʎO�p-�����̏Փˈʒu�����߂�
			///@param	apex1_			�O�p�`�̒��_A
			///@param	apex2_			�O�p�`�̒��_B
			///@param	apex3_			�O�p�`�̒��_C
			///@param	segP_			�����̎n�_
			///@param	segQ_			�����̏I�_
			///@param	normal_			[out]�O�p�`�̖@���A�ȗ��s��
			///@param   distance_		[out]�Փ˓_�̍��W�A�ȗ���
			///@param   intersection_	[out]�Փ˓_�̍��W�A�ȗ���
			///@retval	true			�Փ˂���
			///@retval	false			�Փ˂Ȃ�
			///@note
			///
			/// 	��{�I�ɂ͎O�p�`-���Ɠ���
			/// 	�O�p�`��̓_��uA+vB+wC�ŕ\�����Ƃ�
			/// 		u+v+w=1�Ȃ畽�ʏ�̓_�A�œK���̂���u=1-v-w�ŕ�������������
			/// 		v>0,w>0,v+w<1
			/// 	���ŏI�I�ȏ���
			/// 	��{�I�ɂ͎O�p�`�Ɛ��Ƃ̔���Ɠ����ɂȂ邪�A
			/// 	���̌v�Z�r���ɓ�����e�l���Ɍ��Ă������ƂŁA
			/// 	�Փ˂��Ă��Ȃ��ꍇ�𑁊��ɔ����ł���
			///
			bool TriSegColBothFaces(
				const math::t_vector3&	apex1_,
				const math::t_vector3&	apex2_,
				const math::t_vector3&	apex3_,
				const math::t_vector3&	segP_,
				const math::t_vector3&	segQ_,
				math::t_vector3*		normal_,
				float*					distance_ = NULL,
				math::t_vector3*		intersection_ = NULL);

			///@brief	�O�p�|���S���Ɛ����̍ŋߐړ_
			///@param	apexA1_			�O�p�`�̒��_1
			///@param	apexA2_			�O�p�`�̒��_2
			///@param	apexA3_			�O�p�`�̒��_3
			///@param	segP_			�����̎n�_
			///@param	segQ_			�����̏I�_
			///@param	proximateL_		[out]����̍ŋߐړ_
			///@param	proximateP_		[out]�O�p�`��̍ŋߐړ_
			///@note	�O�p�`�̗��\�̒�`�͍���n�ɏ�����
			void TriSegProximate(
				const math::t_vector3&	apex1_,
				const math::t_vector3&	apex2_,
				const math::t_vector3&	apex3_,
				const math::t_vector3&	segP_,
				const math::t_vector3&	segQ_,
				math::t_vector3*		proximateL_ = NULL,
				math::t_vector3*		proximateP_ = NULL);

			///@brief	���ʎO�p-���ʎO�p�̏Փ˂𔻒肷��
			///@param	apexA1_			�O�p�`A�̒��_1
			///@param	apexA2_			�O�p�`A�̒��_2
			///@param	apexA3_			�O�p�`A�̒��_3
			///@param	apexB1_			�O�p�`B�̒��_1
			///@param	apexB2_			�O�p�`B�̒��_2
			///@param	apexB3_			�O�p�`B�̒��_3
			///@retval	true			�Փ˂��Ă��邩������Ȃ�
			///@retval	false			�m���ɏՓ˂��Ă��Ȃ�
			bool TriTriColPosibility(
				const math::t_vector3&	apexA1_,
				const math::t_vector3&	apexA2_,
				const math::t_vector3&	apexA3_,
				const math::t_vector3&	apexB1_,
				const math::t_vector3&	apexB2_,
				const math::t_vector3&	apexB3_);

		#if 0
		// �ǂ������d�l�ō���Ă������s��
		// �������Ȋ���

			///@brief	�ЖʎO�p-�ЖʎO�p�̏Փ˓_�ƏՓ˕��������߂�
			/// 			�O�p�`�̖�-���̏Փ˂̏ꍇ�ʑ��́B�B�B
			/// 			�ʂƒ��_�̏Փ˂𗼖ʂƂ��Ƃ�B�B�B
			///@param	apexA1_			�O�p�`A�̒��_1
			///@param	apexA2_			�O�p�`A�̒��_2
			///@param	apexA3_			�O�p�`A�̒��_3
			///@param	apexB1_			�O�p�`B�̒��_1
			///@param	apexB2_			�O�p�`B�̒��_2
			///@param	apexB3_			�O�p�`B�̒��_3
			///@param   intersectionA_	[out]�O�p�`A���̏Փ˓_���W�A�ȗ���
			///@param   intersectionB_	[out]�O�p�`B���̏Փ˓_���W�A�ȗ���
			///@param   penetrateA_		[out]�O�p�`A���̐i���ʁA�ȗ���
			///@param   penetrateB_		[out]�O�p�`B���̐i���ʁA�ȗ���
			///@param   penetrateA_		[out]�O�p�`A��B�ɑ΂���i���ʁA�ȗ���
			///@note
			///
			/// 	���̊֐��͍œK�����\���Ǝv����
			/// 	�O�p�|���S���ƎO�p�|���S��
			/// 	�����ŕ�-�ӂ��A���_-�ʂ̏Փ˂����肵�A
			/// 	��-�ӂ̏ꍇ�͂��ꂼ��̍ŋߐړ_�A
			/// 	���_-�ʂ̏ꍇ�͒��_��ʂɓ��e�����_��
			/// 	hitPos1_�AhitPos2_�ɏ������ށB
			///
			bool TriTriCol(
				const math::t_vector3&	apexA1_,
				const math::t_vector3&	apexA2_,
				const math::t_vector3&	apexA3_,
				const math::t_vector3&	apexB1_,
				const math::t_vector3&	apexB2_,
				const math::t_vector3&	apexB3_,
				math::t_vector3*		intersectionA_ = NULL,
				math::t_vector3*		intersectionB_ = NULL,
				math::t_vector3*		penetrateA_ = NULL,
				math::t_vector3*		penetrateB_ = NULL);

			///@brief	���ʎO�p-���ʎO�p�̏Փ˓_�ƏՓ˕��������߂�
			/// 			�O�p�`�̖�-���̏Փ˂̏ꍇ�ʑ��́B�B�B
			/// 			�ʂƒ��_�̏Փ˂𗼖ʂƂ��Ƃ�B�B�B
			///@param	apexA1_			�O�p�`A�̒��_1
			///@param	apexA2_			�O�p�`A�̒��_2
			///@param	apexA3_			�O�p�`A�̒��_3
			///@param	apexB1_			�O�p�`B�̒��_1
			///@param	apexB2_			�O�p�`B�̒��_2
			///@param	apexB3_			�O�p�`B�̒��_3
			///@param   intersectionA_	[out]�O�p�`A���̏Փ˓_���W�A�ȗ���
			///@param   intersectionB_	[out]�O�p�`B���̏Փ˓_���W�A�ȗ���
			///@param   penetrateA_		[out]�O�p�`A���̐i���ʁA�ȗ���
			///@param   penetrateB_		[out]�O�p�`B���̐i���ʁA�ȗ���
			///@param   penetrateA_		[out]�O�p�`A��B�ɑ΂���i���ʁA�ȗ���
			///@note
			///
			/// 	���̊֐��͍œK�����\���Ǝv����
			/// 	�O�p�|���S���ƎO�p�|���S��
			/// 	�����ŕ�-�ӂ��A���_-�ʂ̏Փ˂����肵�A
			/// 	��-�ӂ̏ꍇ�͂��ꂼ��̍ŋߐړ_�A
			/// 	���_-�ʂ̏ꍇ�͒��_��ʂɓ��e�����_��
			/// 	hitPos1_�AhitPos2_�ɏ������ށB
			///
			bool TriTriColBoth(
				const math::t_vector3&	apexA1_,
				const math::t_vector3&	apexA2_,
				const math::t_vector3&	apexA3_,
				const math::t_vector3&	apexB1_,
				const math::t_vector3&	apexB2_,
				const math::t_vector3&	apexB3_,
				math::t_vector3*		intersectionA_ = NULL,
				math::t_vector3*		intersectionB_ = NULL,
				math::t_vector3*		penetrateA_ = NULL,
				math::t_vector3*		penetrateB_ = NULL);
		#endif

			///@brief	�Жʎl�p-�����̏Փˈʒu�����߂�
			///@param	apex1_			�l�p�`�̒��_A
			///@param	apex2_			�l�p�`�̒��_B
			///@param	apex3_			�l�p�`�̒��_C
			///@param	apexD_			�l�p�`�̒��_D
			///@param	segP_			�����̎n�_
			///@param	segQ_			�����̏I�_
			///@param   intersection_	[out]�Փ˓_�̍��W�A�ȗ���
			///@retval	true			�Փ˂���
			///@retval	false			�Փ˂Ȃ�
			///@note
			///
			/// 	�l�p�`�̗��\�̒�`�͍���n�ɏ�����
			/// 	�Ίp���Ő؂��ĎO�p�`�̔��肷��ƁA�Ίp���̂ǂ��瑤�ɂ��邩�ŕ���
			///
			bool SquareSegCol(
				const math::t_vector3&	apex1_,
				const math::t_vector3&	apex2_,
				const math::t_vector3&	apex3_,
				const math::t_vector3&	apexD_,
				const math::t_vector3&	segP_,
				const math::t_vector3&	segQ_,
				math::t_vector3*		intersection_ = NULL);

			#if 0
				///@brief	AABB-�_�̏Փ�
				///@param	halfSize_	AABB�̃{�b�N�X���a
				///@param	center_		AABB�̒��S�_
				///@param	pt_			�_�̍��W
				bool AabbPtCol(
					const math::t_vector3&	center_,
					const math::t_vector3&	halfSize_,
					const math::t_vector3&	pt_);
			#else
				///@brief	AABB-�_�̏Փ�
				///@param	aabb_	AABB
				///@param	pt_		�_�̍��W
				///@retval	true	�Փ˂���
				///@retval	false	�Փ˂Ȃ�
				///@note
				///
				/// 	AABB-�_ �傫��AABB�Ɠ_�Ɏˉe���Čv�Z
				/// 	Axis Aligned Bounding Box �_�Ɏˉe���Čv�Z
				///
				bool AabbPtCol(
					const CAabb3&	aabb_,
					const math::t_vector3&		pt_);

			#endif

			///@brief	AABB-�_�ŋߐړ_���擾����
			///@param	aabb_			AABB
			///@param	p_				�_�̍��W
			///@param	proximate_		[out]�ŋߐړ_
			void AabbPtProximate(
				const CAabb3&     aabb_,
				const math::t_vector3&	p_,
				math::t_vector3*		proximate_);

			///@brief	AABB-���̏Փ˂𔻒肷��
			///@param	aabb_		AABB
			///@param	p_			�_�̍��W
			///@param	radius_		���̔��a
			bool AabbSphereCol(
				const CAabb3&     aabb_,
				const math::t_vector3&	p_,
				float					radius_);

			///@brief	AABB-�����̏Փ˂𔻒肷��
			///@param	aabb_		AABB
			///@param	rayP_		�����̌��_
			///@param	rayDir_		�����̕����ƒ���
			///@param	normal_		[out]�Փ˖ʂ̖@���A�ȗ���
			///@retval	�Փ˓_�܂ł�rayDir_�ɑ΂���W��
			float AabbRayCol(
				const CAabb3&			aabb_,
				const math::t_vector3&	rayP_,
				const math::t_vector3&	rayDir_,
				math::t_vector3*		normal_ = NULL);

			///@brief	AABB-���ʂ̏Փ˂𔻒肷��
			///@param	aabb_		AABB
			///@param	planeN_		���ʂ̖@��
			///@param	planeD_		���ʂ̖@�������I�t�Z�b�g
			///@retval	<0	�{�b�N�X�����ʂ̗���
			///@retval	>0	�{�b�N�X�����ʂ̕\��
			///@retval	==0	���ʂƌ�������
			float AabbPlaneCol(
				const CAabb3&			aabb_,
				const math::t_vector3&	planeN_,
				const float				planeD_);

			///@brief	AABB-�ړ����ʂ̏Փ˂𔻒肷��
			///@param	aabb_		AABB
			///@param	relAabbV_	���ʂɑ΂���AABB�̑��Α��x
			///@param	planeN_		���ʂ̖@��
			///@param	planeD_		���ʂ̖@�������I�t�Z�b�g
			///@retval	<0	�{�b�N�X�����ʂ̗���
			///@retval	>0	�{�b�N�X�����ʂ̕\��
			///@retval	==0	���ʂƌ�������
			float AabbPlaneCol(
				const CAabb3&			aabb_,
				const math::t_vector3&	relAabbV_,
				const math::t_vector3&	planeN_,
				const float&			planeD_);

			#if 0
				///@brief	AABB-AABB
				///@note	Axis Aligned Bounding Box �傫��AABB�Ɠ_�Ɏˉe���Čv�Z����
				///@param	cencterA_	AABB_A�̒��S�_
				///@param	halfSizeA_	AABB_A�̃{�b�N�X���a
				///@param	centerB_	AABB_B�̒��S�_
				///@param	halfSizeB_	AABB_B�̃{�b�N�X���a
				bool AabbAabbCol(
					const math::t_vector3&	cencterA_,
					const math::t_vector3&	halfSizeA_,
					const math::t_vector3&	centerB_,
					const math::t_vector3&	halfSizeB_);
			#else
				///@brief	AABB-AABB�̓��I�Փ˓_�����߂�B
				///@note	AABB_A��ɂ��āAAABB_A�͐Î~���Ă���Ƃ��Čv�Z����
				///@param	aabbA_		AABB_A
				///@param	aabbB_		AABB_B
				///@param	relAabbBV_	A�ɑ΂���B�̑��Α��x
				///@retval	�����J�n����relAabbBV_�ɑ΂���p�����[�^������
				bool AabbAabbCol(
					const CAabb3&		aabbA_,
					const CAabb3&		aabbB_);

				///@brief	AABB-AABB�̓��I�Փ˓_�����߂�B
				///@note	AABB_A��ɂ��āAAABB_A�͐Î~���Ă���Ƃ��Čv�Z����
				///@param	aabbA_		AABB_A
				///@param	aabbB_		AABB_B
				///@param	relAabbBV_	A�ɑ΂���B�̑��Α��x
				///@retval	�����J�n����relAabbBV_�ɑ΂���p�����[�^������
				float AabbAabbCol(
					const CAabb3&		aabbA_,
					const CAabb3&		aabbB_,
					const math::t_vector3&	relVelocityB_);
			#endif

			#if 0
				///@brief		OBB-OBB���Փ˔���ɂ����Ă������̏������Ȃ���
				/// 				�Փ˂���\�������邩�𔻒肷��B<br>
				///@attension	���̊֐���true��Ԃ��Ă��A�Փ˂��Ă��Ȃ��ꍇ������B
				/// 				�����ɔ��肷��ꍇ��ObbObbCol()���g�p���邱�ƁB
				///@param	worldA_		OBB_A�ɓK�p����Ă��郏�[���h�ϊ��s��
				///@param	halfSizeA_	OBB_A�̃{�b�N�X���a
				///@param	worldB_		OBB_B�ɓK�p����Ă��郏�[���h�ϊ��s��
				///@param	halfSizeB_	OBB_B�̃{�b�N�X���a
				///@retval	true		�Փ˂̉\��������
				///@retval	false		�Փ˂Ȃ�
				bool ObbObbColPossibility(
					const math::t_matrix4&	worldA_,
					const math::t_vector3&	halfSizeA_,
					const math::t_matrix4&	worldB_);
					const math::t_vector3&	halfSizeB_,

				///@brief	OBB�|OBB�̏Փ˂𔻒肷��
				///@param	halfSizeA_	OBB_A�̃{�b�N�X���a
				///@param	worldA_		OBB_A�ɓK�p����Ă��郏�[���h�ϊ��s��
				///@param	halfSizeB_	OBB_B�̃{�b�N�X���a
				///@param	worldB_		OBB_B�ɓK�p����Ă��郏�[���h�ϊ��s��
				///@retval	true		�Փ˂���
				///@retval	false		�Փ˂Ȃ�
				bool ObbObbCol(
					const math::t_vector3&	halfSizeA_,
					const math::t_matrix4&	worldA_,
					const math::t_vector3&	halfSizeB_,
					const math::t_matrix4&	worldB_);

				///@brief	��?�Ə����Ă��������{�����H
				// �{��
				// ���b�V���Ƃ��̃��[�J����̓_
				// ��ԋ߂��ʂւ̓��e�Ƃ��̕����̐H�����݂�Ԃ�
				bool MeshPtCol(
					const CTriMesh&			mesh_,
					const math::t_vector3&	p_,			// �Փˈʒu�i���b�V���̃��[�J���j
					float*					depth_,
					math::t_vector3*		n_);


				///@brief	���b�V���Ɛ����Փ˔���Ж�
				// ��ԋ߂��ʂւ̓��e�Ƃ��̕����̐H�����݂�Ԃ�
				bool MeshSegCol(
					const CTriMesh&			mesh_,
					const math::t_vector3&	segP_,					// �����̎n�_
					const math::t_vector3&	segDir_,				// �����̕���// �ƒ����H��������
					float*					depth_);


				///@brief	��?�Ə����Ă��������{�����H
				// �{��
				// ���b�V���Փˈʒu
				// �̐ς��������Փː}�`�ɂȂ��Ă��܂��̂ŁA
				// ���񂩍s�����Ƃŕ��̂����S�ɕ�������
				// colPosA_�FA�̏Փˈʒu�iA���[�J���j
				// colPosB_�FB�̏Փˈʒu�iB���[�J���j
				// penetrate_�F�Փ˕����Ɛ[��(��΍��W)
				bool MeshColPos(
					const CTriMesh&			meshA_,
					const CTriMesh&			meshB_,
					const math::t_matrix4&	worldA_,		// ���ꂼ��̈ʒu�p��
					const math::t_matrix4&	worldB_,
					math::t_vector3*		colPosA_,		// A�̏Փˈʒu�iA���[�J���j
					math::t_vector3*		colPosB_,		// B�̏Փˈʒu�iB���[�J���j
					math::t_vector3*		penetrate_);	// �Փ˕����Ɛ[��(��΍��W)

				///@brief	��?�Ə����Ă��������{�����H
				/// �{��
				/// ���b�V���Փˈʒu
				/// �̐ς��������Փː}�`�ɂȂ��Ă��܂��̂ŁA
				/// ���x�𗘗p���Ă��ꂩ��N����ł��낤
				/// �Փ˂̏u�Ԃ̏Փˏu�Ԃ̈ʒu�����߂�
				bool MeshColPos(
					const CTriMesh&		meshA_,
					const CTriMesh&		meshB_,
					const math::t_matrix4&	worldA_,				// ���ꂼ��̈ʒu�p��
					const math::t_matrix4&	worldB_,
					const math::t_vector3&	vA_,					// ���x
					const math::t_vector3&	vB_,
					const math::t_vector3&	omegaA_,				// �p���x
					const math::t_vector3&	omegaB_,
					math::t_vector3*		colPosA_,				// A�̏Փˈʒu�iA���̒��S����j
					math::t_vector3*		colPosB_,				// B�̏Փˈʒu�iB���̒��S����j
					math::t_vector3*		penetrate_);			// �Փ˕����Ƒ��Α��x
			#endif
			// @}


			///@brief	������-���̏Փ˂𔻒肷��
			///@param	result_    ���ʂ����镽��
			///@param	plane_     ���ɂȂ镽��
			///@param	transform_ �g�����X�t�H�[���s��
			void MakePlaneTransformed(
				CPlane*                 result_,
				const CPlane&			plane_,
				const math::t_matrix4&	transform_);

			///@brief	������-���̏Փ˂𔻒肷��
			///@param	frustum_
			///@param	point_
			///@param	radius_		���ʂ̖@�������I�t�Z�b�g
			///@retval	<0	�{�b�N�X�����ʂ̗���
			///@retval	>0	�{�b�N�X�����ʂ̕\��
			///@retval	==0	���ʂƌ�������
			bool FrustumSphereCol(
				const CFrustum&			frustum_,
				const math::t_vector3&	point_,
				float				    radius_);

			///@attention ����ō����������̓T�C�h�̕��ʁim_sides�j�ȊO�̃����o�[�͖����ł���
			///@brief	������̃g�����X�t�H�[��
			///@param	result_    ���ʂ����鎋����
			///@param	frustum_   ���ɂȂ鎋����
			///@param	transform_ �g�����X�t�H�[���s��
			void MakeFrustumTransformed(
				CFrustum*               result_,
				const CFrustum&			frustum_,
				const math::t_matrix4&	transform_);


		// }
	}
}
#endif
	