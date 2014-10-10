/// rflab. All Rights Reserved.
///@file	collisiondetectfunction.cpp
///@auther	rflab.
///@date	2010/06/24
///@brief	�Փ˔���֐�

#include <bighead.h>
#include <rflib/common/index.h>
#include "collisiondetectfunction.h"
#include <cassert>
#include <algorithm>


using namespace rf;
using namespace collision;
using namespace math;


namespace rf
{
	namespace collision
	{
		// ------------------------------------------------------------------
		// 2d
		// ------------------------------------------------------------------

		///@brief	�_-�_�̋��������߂�
		///@param	a_	�_A�̍��W
		///@param   b_	�_B�̍��W
		///@retval  �_-�_�̋���
		float PtPtDistance(
			const t_vector2&	a_,
			const t_vector2&	b_)
		{
			t_vector2 ab(b_ - a_);
			return ab.Length();
		}

		///@brief	�~-�~�̏Փ˂𔻒肷��
		///@param	centerA_	�~A�̍��W
		///@param	radiusA_	�~A�̍��W
		///@param   centerB_	�~B�̍��W
		///@param   radiusB_	�~B�̍��W
		///@note
		///@retval  �_-�_�̋���
		float CircleCircleCol(
			const t_vector2&	centerA_,
			float 				radiusA_,
			const t_vector2&	centerB_,
			float 				radiusB_)
		{
			t_vector2 ab(centerA_ - centerB_);
			return ab.Length() < radiusA_ + radiusB_;
		}

		///@brief	AABB-�_�̏Փ�
		///@param	aabb_	AABB
		///@param	pt_		�_�̍��W
		///@retval	true	�Փ˂���
		///@retval	false	�Փ˂Ȃ�
		bool AabbPtCol(
			const CAabb2&			aabb_,
			const math::t_vector2&	pt_)
		{
			return	pt_(0) <= aabb_.mini(0) && pt_(0) <= aabb_.mini(0) &&
					pt_(1) <= aabb_.mini(1) && pt_(1) <= aabb_.mini(1);
		}

		///@brief	AABB-�_�ŋߐړ_���擾����
		///@param	aabb_			AABB
		///@param	p_				�_�̍��W
		///@param	proximate_		[out]�ŋߐړ_
		void ::rf::collision::AabbPtProximate(
			const CAabb2&			aabb_,	
			const math::t_vector2&	p_,
			math::t_vector2*		proximate_)
		{
			for (int i=0; i<2; i++)
			{
				if (p_(i) < aabb_.mini(i))
					(*proximate_)(i) = aabb_.mini(i);
				else if (aabb_.maxi(i) < p_(i))
					(*proximate_)(i) = aabb_.maxi(i);
				else
					(*proximate_)(i) = p_(i);		
			}
		}

		///@brief	AABB-AABB�̏Փ˂𔻒肷��
		///@param	aabbA_	AABB_A
		///@param	aabbB_	AABB_B
		///@retval	true		�Փ˂���
		///@retval	false		�Փ˂Ȃ�
		///@note	center-size�^��AABB�Ȃ�A�傫��AABB�Ɠ_��
		/// 			�ˉe���ďՓ˂𔻒肵�������������B
		/// 			mini-maxi�^��AABB�Ȃ炻�̌���ł͂Ȃ��H
		bool AabbAabbCol(
			const CAabb2&	aabbA_,
			const CAabb2&	aabbB_)
		{
			for (int i=0; i<2; i++)
			{
				if (aabbA_.mini(i) > aabbB_.maxi(i))
					return false;
				if (aabbA_.maxi(i) > aabbB_.mini(i))
					return false;
			}
			return true;
		}

		// ------------------------------------------------------------------
		// 3d
		// ------------------------------------------------------------------

		///@brief	��]�̂̈ʒu�Ƒ��x�Ɗp���x����A����_�̑��x�����߂�
		///@param	world_		��]�̂̃��[���h�ϊ��s��
		///@param   v_			��]�̂̑��x
		///@param	w_			��]�̂̊p���x	
		///@param	p_			���x�����߂����_�̉�]�̃��[�J�����W
		///@param   sweepVec_	[out]���[���h�ł̓_�̑��x
		void RotSweep(
			const t_matrix4&	T_,		// ��]���S�̈ʒu�A���݂̎p��
			const t_vector3&	r_,		// ��]���S����̃��[�J�����W�ʒu
			const t_vector3&	v_,		// ���x
			const t_vector3&	w_,		// �p���x
			t_vector3*			sweepVec_)	// ���ʁF�X�C�[�v�̃x�N�g��
		{
			// ���[�J���ł̊p���x
			t_vector3 relW;
			MakeVectorCoordinateTransformed(&relW, T_, w_);

			// r�̓_�̉�]�ɂ�鑬�x
			t_vector3 v;
			MakeVectorCross(&v, r_, relW);

			// ��]�ɂ�鑬�x�����[���h�ɕϊ�
			(*sweepVec_)(0) = v(0)*T_(0, 0);
			(*sweepVec_)(1) = v(1)*T_(1, 0);
			(*sweepVec_)(2) = v(2)*T_(2, 0);
	
			// ���i�ɂ�鑬�x�����Z
			*sweepVec_ += v_;
		}


		///@brief	�_-�_�̋��������߂�
		///@param	a_	�_A�̍��W
		///@param   b_	�_B�̍��W
		///@retval  �_-�_�̋���
		float PtPtDistance(
			const t_vector3&	A_,
			const t_vector3&	B_)
		{
			t_vector3 ab = A_ - B_;
			return ab.Length();
		}

		///@brief	�~-�~�̏Փ˂𔻒肷��
		///@param	centerA_	�~A�̒��S���W
		///@param	radiusA_	�~A�̔��a
		///@param   centerB_	�~B�̒��S���W
		///@param   radiusB_	�~B�̔��a
		///@retval  �_-�_�̋���
		float SphereSphereCol(
			const t_vector3&	centerA_,
			float 				radiusA_,
			const t_vector3&	centerB_,
			float 				radiusB_)
		{
			t_vector3 ab(centerA_ - centerB_);
			float r = radiusA_ + radiusB_;

			// �Q��l�Ŕ�r�����������
			return ab.LengthSquared() < r*r;
		}


		///@brief	�_-���̋��������߂�
		///@param	p_			�_�̍��W
		///@param	lineP_		����̔C�ӂ̓_
		///@param   lineDir_	���K�����ꂽ���̕���
		///@retval  �_-���̋���
		float LinePtDistance(
			const t_vector3&	p_,					
			const t_vector3&	lineP_,
			const t_vector3&	lineDir_)
		{
			// �����x�N�g���͐��K������Ă��邩�`�F�b�N
			assert(lineDir_.Length() - 1.0 < 0.01f);

			t_vector3 pp = p_ - lineP_;
			t_vector3 cross = Cross(pp, lineDir_);
			float area = cross.Length();

			return fabs(area*lineDir_.ReciprocalLength());
		}

		///@brief	������ɂ���_p_�̍ŋߐړ_�����߂�
		///@param	p_			�_�̍��W
		///@param	segP_		�����̎n�_
		///@param   segQ_		�����̏I�_
		///@param   proximate_	[out]�ŋߐړ_
		void ::rf::collision::SegPtProximate(				
			const t_vector3&	segP_,				
			const t_vector3&	segQ_,
			const t_vector3&	p_,					
			t_vector3*			proximate_)
		{
#if 0
			t_vector3 pp = p_ - segP_;
			t_vector3 pq(segQ_-segP_);
			t_vector3 dir;
			MakeVectorNormalized(&dir, pq);

			float t = Dot(pp, pq);
			*proximate_ = dir * t;
#endif
			
			if (Dot(t_vector3(segQ_-segP_), t_vector3(p_-segP_)) < 0)
			{
				*proximate_ = segP_;
			}	
			else if (Dot(t_vector3(segP_-segQ_), t_vector3(p_-segQ_)) < 0)
			{	
				*proximate_ = segQ_;
			}
			else
			{
				float dir_len = Length(t_vector3(segQ_-segP_));
				t_vector3 dir = t_vector3(segQ_-segP_) / dir_len;
				//MakeVectorNormalized(&dir, t_vector3(segQ_-segP_));
				//float t = Length(Cross(t_vector3(segQ_-segP_), t_vector3(p_-segP_)))
				//	/ Length(t_vector3(segQ_-segP_));
				float t = Dot(dir, t_vector3(p_-segP_));
				*proximate_ = dir * t + segP_;
			}
			return;
		}

		#if 0
			///@brief	������ɂ���_p_�̍ŋߐړ_�����߂�
			///@param	p_			�_�̍��W
			///@param	segP_		�����̎n�_
			///@param   segQ_		�����̏I�_
			///@param   proximate_	[out]�ŋߐړ_
			void SegPtProximate(				
				const t_vector3&	segP_,				
				const t_vector3&	segQ_,
				const t_vector3&	p_,					
				t_vector3*			proximate_)
			{
				t_vector3 pqDir;
				t_vector3 pp = p_ - segP_;
				MakeVectorNormalized(t_vector3(segQ_ - segP_), &pqDir);
				float t = Dot(pp, pqDir);
				Clamp(&t, 0, 1);
				*proximate_ = t*pqDir;
			}
		#endif

		///@brief	�_-�����̋��������߂�
		///@param	p_			�_�̍��W
		///@param	segP_		�����̎n�_
		///@param   segQ_		�����̏I�_
		///@retval	�_-�����̋���
		float SegPtDistance(
			const t_vector3&	p_,					
			const t_vector3&	segP_,	
			const t_vector3&	segQ_)
		{
			#if 0
				t_vector3 ab = p_ - segP_;					
				t_vector3 ac = segQ_ - segP_;		
				t_vector3 bc = p_- segQ_;
				float e = Dot(ac, ab);
				if(e < 0)
					return ac.Length();
				float f = Dot(ab, ab);
					return bc.Length();
			#else
				t_vector3 proximate;
				SegPtProximate(segP_, segQ_, p_, &proximate);
				return Length(t_vector3(proximate - p_));
			#endif
		}

		///@brief	��-���̋��������߂�
		///@param	segAP_		��A�̎n�_
		///@param	segADir_	��A�̏I�_
		///@param   segBP_		��B�̎n�_
		///@param   segBDir_	��B�̏I�_
		///@retval	��-���̋���
		float LineLineDistance(
			const t_vector3&	segAP_,		
			const t_vector3&	segADir_,	
			const t_vector3&	segBP_,		
			const t_vector3&	segBDir_)
		{
			t_vector3 pp = segBP_ - segAP_;
			t_vector3 cross = Cross(segADir_, segBDir_);
			return Dot(cross, pp)*ReciprocalLength(cross);
		}

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
			const t_vector3&	lineAP_,		
			const t_vector3&	lineADir_,		
			const t_vector3&	lineBP_,		
			const t_vector3&	lineBDir_,
			t_vector3*			proximateA_,	
			t_vector3*			proximateB_)
		{
			t_vector3 pp = lineAP_ - lineBP_;
			float a = Dot(lineADir_, lineADir_);
			float b = Dot(lineADir_, lineBDir_);
			float c = Dot(lineADir_, pp);
			float e = Dot(lineBDir_, lineBDir_);
			float f = Dot(lineBDir_, pp);
			float d = a*e-b*b;
			float s = (b*f-c*e)/d;
			float t = (a*f-b*c)/d;
			*proximateA_ = lineAP_ + s*lineADir_;
			*proximateB_ = lineBP_ + t*lineBDir_;
		}

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
		void ::rf::collision::SegSegProximate(
			const t_vector3&	segAP_,			
			const t_vector3&	segAQ_,				
			const t_vector3&	segBP_,				
			const t_vector3&	segBQ_,
			t_vector3*		proximateA_,
			t_vector3*		proximateB_)
		{
			t_vector3 dirA = segAQ_ - segAP_;
			t_vector3 dirB = segBQ_ - segBP_;
			t_vector3 pp = dirA - dirB;
			float a = Dot(dirA, dirA);
			float b = Dot(dirA, dirB);
			float c = Dot(dirA, pp);
			float e = Dot(dirB, dirB);
			float f = Dot(dirB, pp);
			float d = a*e-b*b;
			float s = (b*f-c*e)/d;
			float t = (a*f-b*c)/d;
	
			Clamp(&s, 0.0f, 1.0f);
			t = (b*s+f)/e;
			if(t < 0)
			{
				t = 0;
				s = -c/a;
				Clamp(&s, 0.0f, 1.0f);
			}
			if(t > 1.0f)
			{
				t = 1.0f;
				s = (b-c)/a;
				Clamp(&s, 0.0f, 1.0f);
			}
			*proximateA_ = s*dirA;
			*proximateB_ = t*dirB;
		}

		///@brief	����-�_�̈ʒu�֌W�𔻒肷��
		///@param	planeP_		���ʏ�̔C�ӂ̓_
		///@param	planeN_		���ʂ̖@��
		///@param   p_			�_�̍��W
		///@retval	true		�\��
		///@retval	false		����
		bool PlanePtClassifyInside(
			const t_vector3&	planeP_,		
			const t_vector3&	planeN_,		
			const t_vector3&	p_)			
		{
			t_vector3 pp = p_ - planeP_;
			return Dot(pp, planeN_) < 0;
		}

		///@brief	����-�_�̈ʒu�֌W�𔻒肷��
		///@param	apex1_		���ʏ�̔C�ӂ̓_A
		///@param	apex2_		���ʏ�̔C�ӂ̓_B
		///@param	apex3_		���ʏ�̔C�ӂ̓_C
		///@param   p_			�_�̍��W
		///@retval	true		�\��
		///@retval	false		����
		///@note	���ʂ̗��\�̒�`�͍���n�ɏ�����
		bool PlanePtClassifyInside(
			const t_vector3&	apex1_,			
			const t_vector3&	apex2_,
			const t_vector3&	apex3_,
			const t_vector3&	p_)		
		{
			t_vector3 n;
			t_vector3 ab = apex2_ - apex1_;
			t_vector3 ac = apex3_ - apex1_;
			t_vector3 ap = p_ - apex1_;

			return Dot(ap, n) < 0;
		}

		///@brief	���ʂƓ_�̕����t���������߂�
		///@param	plane_		����
		///@param	p_			���ʂ̖@��
		///@param   depth_		[out]���ʂƓ_�̕����t����
		///@retval  ���ʂƓ_�̕����t�����A�����\��
		///@note	���ʂ̗��\�̒�`�͍���n�ɏ�����
		float ::rf::collision::PlanePtDistance(
			const CPlane&			plane_,
			const math::t_vector3&	p_)
		{
			float d = Dot(plane_.normal(), p_);
			return d - plane_.d();
		}

		///@brief	���ʂƓ_�̕����t���������߂�
		///@param	planeP_		���ʏ�̔C�ӂ̓_
		///@param	planeN_		���ʂ̖@��
		///@param	p_			���ʂ̖@��
		///@param   depth_		[out]���ʂƓ_�̕����t����
		///@retval  ���ʂƓ_�̕����t�����A�����\��
		///@note	���ʂ̗��\�̒�`�͍���n�ɏ�����
		///@note	�{����P�̈ʒu��������Ȃ��Ă����������܂�̂ŏ�����
		float ::rf::collision::PlanePtDistance(
			const math::t_vector3&	planeP_,			
			const math::t_vector3&	planeN_,
			const math::t_vector3&	p_)
		{
			math::t_vector3 normal;

			// �@��	
			MakeVectorNormalized(&normal, planeN_);
	
			// �@�������̓_�̈ʒu
			float p = Dot(p_, normal);

			// �@�������̕��ʏ�̓_�̈ʒu
			float d = Dot(planeP_, normal);

			return d - p;
		}

		///@brief	���ʂƓ_�̕����t���������߂�
		///@param	apex1_		���ʏ�̔C�ӂ̓_A
		///@param	apex2_		���ʏ�̔C�ӂ̓_B
		///@param	apex3_		���ʏ�̔C�ӂ̓_C
		///@param	p_			���ʂ̖@��
		///@param   depth_		[out]���ʂƓ_�̕����t����
		///@param   normal_		[out]���ʂ̖@��
		///@note	���ʂ̗��\�̒�`�͍���n�ɏ�����
		float ::rf::collision::PlanePtDistance(
			const t_vector3&	apex1_,	
			const t_vector3&	apex2_,
			const t_vector3&	apex3_,
			const t_vector3&	p_,		
			float*				depth_,		
			math::t_vector3*	normal_)
		{
			t_vector3 penetrate_;
			t_vector3 ab = apex2_ - apex1_;
			t_vector3 ac = apex3_ - apex1_;
			t_vector3 ap = p_ - apex1_;
	
			// �@��
			MakeVectorCross(normal_, ab, ac);		
			normal_->Normalize();

			// �H�����݂̓��e
			*depth_ = Dot(ap, *normal_);

			return *depth_;
		}
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
			const t_vector3&	planeP_,				
			const t_vector3&	planeN_,				
			const t_vector3&	segP_,					
			const t_vector3&	segQ_,					
			t_vector3*			intersection_)
		{
			// ���Έʒu�ɕϊ�
			t_vector3 p1 = segP_ - planeP_;
			t_vector3 p2 = segQ_ - planeP_;
	
			float distanceS = Dot(p1, planeN_);
			float distanceE = Dot(p2, planeN_);

			if(distanceS * distanceE > 0)
				return false; 
	
			if(intersection_ == NULL)
				return true;

			*intersection_ = (distanceS / 
				(distanceS+distanceE)) * (p2 - p1);
			return true;
		}

		///@brief	���ʂƔ������̏Փ˂𔻒肷��
		///@param	planeP_			���ʏ�̔C�ӂ̓_
		///@param	planeN_			���ʂ̖@��
		///@param	rayP_,			�������̎n�_
		///@param	rayDir_			�������̏I�_
		///@param   intersection_	[out]�Փ˓_�̍��W�A�ȗ���
		///@retval	true			�Փ˂���
		///@retval	false			�Փ˂Ȃ�
		// ��ƂقƂ�Ǔ���
		bool PlaneRayCol(
			const t_vector3&	planeP_,				
			const t_vector3&	planeN_,				
			const t_vector3&	rayP_,					
			const t_vector3&	rayVec_,				
			t_vector3*			intersection_)	
		{
			// ���Έʒu�ɕϊ�
			t_vector3 p1 = rayP_ - planeP_;
			t_vector3 p2 = rayVec_ + p1;
	
			float distanceS = Dot(p1, planeN_);
			float distanceE = Dot(p2, planeN_);

			if(distanceS * distanceE > 0)
				return false; 
	
			if(intersection_ == NULL)
				return true;

			*intersection_ = (distanceS /
				(distanceS+distanceE)) * (p2 - p1);
			return true;
		}

		///@brief	�ЖʎO�p-���̏Փˈʒu�����߂�
		///@param	apex1_			�O�p�`�̒��_A
		///@param	apex2_			�O�p�`�̒��_B
		///@param	apex3_			�O�p�`�̒��_C
		///@param	lineP_			����̔C�ӂ̓_
		///@param	lineDir_		���̕���
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
			const t_vector3&	apex1_,					
			const t_vector3&	apex2_,
			const t_vector3&	apex3_,
			const t_vector3&	lineP_,					
			const t_vector3&	lineDir_,
			t_vector3*			normal_,
			t_vector3*			intersection_)
		{
			t_vector3 vecPA = apex1_ - lineP_;
			t_vector3 vecPB = apex2_ - lineP_;
			t_vector3 vecPC = apex3_ - lineP_;
			t_vector3 vecPQ = lineDir_;

			// �X�J���O�d��
			// �����@���ɂ��QP�~PC���g���܂킷
			t_vector3 cross;
			MakeVectorCross(&cross, vecPQ, vecPC);

			float u = Dot(cross, vecPB);
			if(u < 0)
				return false;
			float v = - Dot(cross, vecPA);
			if(v < 0)
				return false;
			float w = ScalarTriple(vecPQ, vecPB, vecPA);
			if(w < 0)
				return false;
	
			// �������O�p���ʏザ��Ȃ���
			if(u<0.001f && v<0.001f && w<0.001f)
				return false;
	
			if (normal_ != NULL)
				MakeVectorNormalized(normal_, cross);

			if (intersection_ != NULL)
				*intersection_ = (u*vecPA+v*vecPB+w*vecPC)/(u+v+w)+lineP_;

			return true;
		}

		///@brief	�ЖʎO�p-�����̏Փˈʒu�����߂�
		///@param	apex1_			�O�p�`�̒��_A
		///@param	apex2_			�O�p�`�̒��_B
		///@param	apex3_			�O�p�`�̒��_C
		///@param	segP_			�����̎n�_
		///@param	segQ_			�����̏I�_
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
			const t_vector3&	apex1_,
			const t_vector3&	apex2_,
			const t_vector3&	apex3_,
			const t_vector3&	segP_,
			const t_vector3&	segDir_,
			math::t_vector3*	normal_,
			float*				distance_,
			math::t_vector3*	intersection_)	
		{
			t_vector3 vecAB = apex2_ - apex1_;
			t_vector3 vecAC = apex3_ - apex1_;

			// ���ʌ�������
			t_vector3 normal; 
			MakeVectorCross(&normal, vecAB, vecAC);

			// QP�~n�ɂ������̑O����
			// QP�̖@����������d>0�Ȃ�����Ⴂ
			// d�ɂ�鐳�K���͂��Ȃ��Ŋ�l�Ƃ��Ĉ���
			float d = Dot(segDir_, normal);
			if(d > 0)
				return false; 

			// AP��QP��@�������ɓ��e�����p�����[�^�����̕������r
			t_vector3 vecAP(segP_ - apex1_);
			float t = Dot(vecAP, normal);

			// ���łɊђʂ��Ă���
			if(t < 0)
				return false;

			// |AP�En| < |d|�Ȃ�����͂��Ȃ�
			if(t > -d)
				return false;

			// �X�J���O�d�ςɂ���������
			t_vector3 e = Cross(segDir_, vecAP);
		
			// v = (C-A)�Ee/d�ɂ��A
			// v��d���|���Đ��ɂȂ�\��Ȃ̂ŕ��ł���K�v������
			float v = Dot(vecAC, e);
			if(0 < v || v < d)
				return false;
			float w = - Dot(vecAB, e);
			if(0 < w || v+w < d)
				return false;
	
			// �����O�p�`��
			if(v < 0.01f && w < 0.01f)
				return false;

			if (distance_ != NULL)
				*distance_ = t / -d;

			if (normal_ != NULL)
				MakeVectorNormalized(normal_, normal); 		

			// v /= d;
			// w /= d;
			// u /= 1 - v - w;

			return true;
		}


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
		/// 	���ʂ̕����኱�������d��
		///
		bool TriSegColBothFaces(
			const t_vector3&	apex1_,					
			const t_vector3&	apex2_,
			const t_vector3&	apex3_,
			const t_vector3&	segP_,					
			const t_vector3&	segQ_,					
			t_vector3*			normal_,
			float*				distance_,
			t_vector3*			intersection_)
		{
			// �e��x�N�g��
			t_vector3 vecAB = apex2_ - apex1_;
			t_vector3 vecAC = apex3_ - apex1_;
			t_vector3 vecPQ = segQ_ - segP_;

			// �@��
			t_vector3 normal; 
			MakeVectorCross(&normal, vecAB, vecAC);

			float d = Dot(vecPQ, normal);
	
			// �������O�p���ʂɕ��s�ȏꍇ��r��
			// �Жʂ̏ꍇ�͕�������ő����I����
			if(fabs(d) < 0.01f)
				return false;

			// AP��QP�̖@�����������̕������r
			// �����ꍇ�͌�����������������
			// |AP�En| < |d|�Ȃ�����͂��Ȃ�
			t_vector3 vecAP(segP_ - apex1_);
			float t = Dot(vecAP, normal);
			if ((fabs(t) > fabs(d) + RF_EPSILON)
			||  (t*d > 0))
				return false;

			// ���O�ɉ���������p����t,v,w�����߂�B
			t_vector3 e; // �v�Z�p
			MakeVectorCross(&e, vecPQ, vecAP); 

			// v = (C-A)�Ee/d
			float v = Dot(vecAC, e);
			if(v*d < 0 || fabs(v) > fabs(d)+RF_EPSILON)
				return false;

			// v = (B-A)�Ee/d
			float w = - Dot(vecAB, e);
			if(w*d < 0 || fabs(v+w) > fabs(d)+RF_EPSILON)
				return false;

			// v /= d;
			// w /= d;
			// u /= 1 - v - w; 

			if(distance_ != NULL)
				*distance_ = t / -d;

			if(intersection_ != NULL)
				*intersection_ = t*vecPQ + segP_;

			if(normal_ != NULL)
				MakeVectorNormalized(normal_, normal);

			return true;
		}

		///@brief	�O�p�|���S���Ɛ����̍ŋߐړ_
		///@param	apexA1_			�O�p�`�̒��_1
		///@param	apexA2_			�O�p�`�̒��_2
		///@param	apexA3_			�O�p�`�̒��_3
		///@param	segP_			�����̎n�_
		///@param	segQ_			�����̏I�_
		///@param	proximateL_		[out]����̍ŋߐړ_
		///@param	proximateP_		[out]�O�p�`��̍ŋߐړ_
		///@note
		///
		/// 	�O�p�`�̗��\�̒�`�͍���n�ɏ�����
		/// 	�O�p�|���S���Ɛ����̍ŋߐړ_
		/// 	�ӑΐ����i3��ށj�ƁA�O�p�`�ΐ��̒[�_�i2��ށj
		/// 	�̂����A�����Ƃ������̒Z���������́B
		/// 	�ӑΐ����̍ŋߐړ_��S���̕ӂɂ��ċ��߁A�ŏ��̂��̂����߂�
		/// 	�ӂƕӂ̋��������߂āA�ŏ��ɂ��Ă����ŋߐړ_�����߂��ق����������H
		///
		void TriSegProximate(
			const t_vector3&	apex1_,			
			const t_vector3&	apex2_,
			const t_vector3&	apex3_,
			const t_vector3&	segP_,			
			const t_vector3&	segQ_,			
			t_vector3*			proximateL_,		
			t_vector3*			proximateP_)		
		{
			t_vector3 PQ = segQ_ - segP_;
			t_vector3 proximateL;
			t_vector3 proximateP; 
			float mini;
		
			// �ӂP
			SegSegProximate(segP_, segQ_, apex1_, apex2_, proximateL_, proximateP_);
			t_vector3 proximate = *proximateP_ - *proximateL_;
			mini = Length(proximate);
	
			// �ӂQ
			SegSegProximate(segP_, segQ_, apex2_, apex3_, &proximateL, &proximateP);
			proximate = proximateP - proximateL;
			float len = proximate.Length();
			if(mini > len)
			{
				mini = len;
				*proximateL_ = proximateL;
				*proximateP_ = proximateP;
			}

			// �ӂR
			SegSegProximate(segP_, segQ_, apex3_, apex1_, &proximateL, &proximateP);
			proximate = proximateP - proximateL;
			len = proximate.Length();
			if(mini > len)
			{
				mini = len;
				*proximateL_ = proximateL;
				*proximateP_ = proximateP;
			}
		}
		// �O�p�|���S���ƎO�p�|���S����Փ˂��Ă��Ȃ����Ƃ𔻒�
		// �Փ˂��Ă��Ȃ����true
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
			const t_vector3&	apexA1_,				// �|���S���̒��_
			const t_vector3&	apexB1_,
			const t_vector3&	apexC1_,
			const t_vector3&	apexA2_,				// �|���S���̒��_
			const t_vector3&	apexB2_,
			const t_vector3&	apexC2_)
		{
			t_vector3 n;
			float dot;

			// �P��
			t_vector3 tmp1 = apexB1_ - apexA1_;
			t_vector3 tmp2 = apexC1_ - apexA1_;
			t_vector3 tmp3 = apexA2_ - apexA1_;
			t_vector3 tmp;
			MakeVectorCross(&n, tmp1, tmp2);
			dot = Dot(n, tmp3);
			if(dot>=0)
			{
				tmp = apexB2_ - apexA1_;
				if(Dot(n, tmp)>=0)
				{
					tmp = apexC2_ - apexA1_;
					if(Dot(n, tmp)>=0)
						return true;
				}
			}
			else// dot1<0
			{
				tmp = apexB2_ - apexA1_;
				if(Dot(n, tmp)>=0)
				{
					tmp = apexC2_ - apexA1_;
					if(Dot(n, tmp)>=0)
						return true;
				}
			}

			// 2��
			tmp1 = apexB2_ - apexA2_;
			tmp2 = apexC2_ - apexA2_;
			tmp3 = apexA1_ - apexA2_;
			MakeVectorCross(&n, tmp1, tmp2);
			dot = Dot(n, tmp3);
			if(dot>=0)
			{
		
				tmp = apexB1_ - apexA2_;
				if(Dot(n, tmp)>=0)
				{
					tmp = apexC1_ - apexA2_;
					if(Dot(n, tmp)>=0)
						return true;
				}
			}
			else
			{
				// dot1<0

				tmp = apexB1_ - apexA2_;
				if(Dot(n, tmp)<0)
				{
					tmp = apexC1_ - apexA2_;
					if(Dot(n, tmp)<0)
						return true;
				}
			}
		
			return false;

		}


		#if 0

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
		e_collide_bits TriTriCol(
			const t_vector3&	apexA1_,				
			const t_vector3&	apexA2_,
			const t_vector3&	apexA3_,
			const t_vector3&	apexB1_,				
			const t_vector3&	apexB2_,
			const t_vector3&	apexB3_,
			math::t_vector3*	intersectionA_ = NULL,	
			math::t_vector3*	intersectionB_ = NULL,
			math::t_vector3*	penetrate_ = NULL);	
		{
			enum HIT_FLAG			// for �O�p�`�ΎO�p�`
			{
				HIT_AB1 = 1,
				HIT_BC1 = 1<<1,
				HIT_CA1 = 1<<2,
				HIT_AB2 = 1<<3,
				HIT_BC2 = 1<<4,
				HIT_CA2 = 1<<5,
			};

			// �ŏ��ɁA�ǂ��Փ˂��邩�𒲂ׂ�
			int hitCount1 = 0;
			int hitFlag1 = 0;
			int hitCount2 = 0;
			int hitFlag2 = 0;

			// �P�̖ʂɑ΂���Q�̕�
			if(TriSegColBothFaces(apexA1_, apexA2_, apexA3_, apexB1_, apexB2_, NULL)==true)
			{
				hitFlag1 |= HIT_AB2;
				hitCount1++;
			}
			if(TriSegColBothFaces(apexA1_, apexA2_, apexA3_, apexB2_, apexB3_, NULL)==true)
			{
				hitFlag1 |= HIT_BC2;
				hitCount1++;
			}
			if(hitCount1 < 2)
			if(TriSegColBothFaces(apexA1_, apexA2_, apexA3_, apexB3_, apexB1_, NULL)==true)
			{
				hitFlag1 |= HIT_CA2;
				hitCount1++;
			}

			// �Q�̖ʂɑ΂���P�̕�
			if(TriSegColBothFaces(apexB1_, apexB2_, apexB3_, apexA1_, apexA2_, NULL)==true)
			{
				hitFlag2 |= HIT_AB1;
				hitCount2++;
			}
			if(TriSegColBothFaces(apexB1_, apexB2_, apexB3_, apexA2_, apexA3_, NULL)==true)
			{
				hitFlag2 |= HIT_BC1;
				hitCount2++;
			}
			if(hitCount2 < 2)
			if(TriSegColBothFaces(apexB1_, apexB2_, apexB3_, apexA3_, apexA1_, NULL)==true)
			{
				hitFlag2 |= HIT_CA1;
				hitCount2++;
			}
	
			// �q�b�g�J�E���g2�͖ʑΒ��_
			if(hitCount1 == 2 && hitCount2 == 0)
			{
				float depth;
				t_vector3 n;
				t_vector3 tmp1 = apexA2_-apexA1_;
				t_vector3 tmp2 = apexA3_-apexA1_;
				Cross(tmp1, tmp2, &n);
				n.Normalize();
				switch(hitFlag1)
				{
				case HIT_AB2 | HIT_BC2:
					tmp1 = apexB2_ - apexA1_;
					depth = Dot(tmp1, n);
					if(depth > 0)
						return RF_COLLIDE_NONE;
					if (hitPos1_ != NULL)
						*hitPos1_ = apexB2_
					if (hitPos2_ != NULL)
						*hitPos2_ = apexB2_;

					break;

				case HIT_CA2 | HIT_BC2:
					tmp1 = apexB3_ - apexA1_;
					depth = Dot(tmp1, n);
					if(depth > 0)
						return RF_COLLIDE_NONE;
					if (hitPos1_ != NULL)
						*hitPos1_ = apexB3_
					if (hitPos2_ != NULL)
						*hitPos2_ = apexB3_;
					break;

					#if 1
						// case HIT_AB2 | HIT_CA2:
						default:
							tmp1 = apexB1_ - apexA1_;
							depth = Dot(tmp1, n);
							if(depth > 0)
								return RF_COLLIDE_NONE;
							if (hitPos1_ != NULL)
								*hitPos1_ = apexB1_
							if (hitPos2_ != NULL)
								*hitPos2_ = apexB1_;
							break;

					#else
						case HIT_AB2 | HIT_CA2:
							tmp1 = apexB1_ - apexA1_;
							depth = Dot(tmp1, n);
							if(depth > 0)
								return RF_COLLIDE_NONE;
							if (hitPos1_ != NULL)
								*hitPos1_ = apexB1_
							if (hitPos2_ != NULL)
								*hitPos2_ = apexB1_;
							break;

						default:
							depth=0;
							RFASSERT(false, NULL);
					#endif
				}
				*penetrate_ = n * depth;
				return RF_COLLIDE_VERTEX;
			}

			// �q�b�g�J�E���g2�͖ʑΒ��_
			if(hitCount2 == 2 && hitCount1 == 0)
			{
				float depth = 0;// �x���΍�ɂO�ŏ�����
				t_vector3 n;
				t_vector3 tmp1 = apexB2_-apexA2_;
				t_vector3 tmp2 = apexC2_-apexA2_;
				Cross(&n, &tmp1, &tmp2);	// �@��
				n = n /D3DXVec3Length(&n);
				switch(hitFlag2)
				{
				case HIT_AB1 | HIT_BC1:
					tmp1 = apexA2_ - apexB1_;
					depth = Dot(&tmp1, &n);
					if(depth > 0)
						return RF_COLLIDE_NONE;
					if (hitPos1_ != NULL)
						*hitPos1_ = apexA2_;
					if (hitPos2_ != NULL)
						*hitPos2_ = apexA2_;
					break;

				case HIT_CA1 | HIT_BC1:
					tmp1 = apexA3_ - apexB1_;
					depth = Dot(&tmp1, &n);
					if(depth > 0)
						return RF_COLLIDE_NONE;
					if (hitPos1_ != NULL)
						*hitPos1_ = apexA3_;
					if (hitPos2_ != NULL)
						*hitPos2_ = apexA3_;
					break;

				case HIT_AB1 | HIT_CA1:
					tmp1 = apexA1_ - apexB1_;
					depth = Dot(&tmp1, &n);
					if(depth > 0)
						return RF_COLLIDE_NONE;
					if (hitPos1_ != NULL)
						*hitPos1_ = apexA1_;
					if (hitPos2_ != NULL)
						*hitPos2_ = apexA1_;
					break;

				default:
					RFASSERT(false, NULL);
				}
				*penetrate_ = n * -depth;

				return RF_COLLIDE_VERTEX;
			}

			// �ӂ̏Փ�
			// ���_-�ӂ��܂߂�
			if(hitCount1 > 0 && hitCount2 > 0)
			{
				int flag = hitFlag2|hitFlag1;
		
				if((flag&(HIT_AB1|HIT_AB2)) == (HIT_AB1|HIT_AB2))
					LineLineProximate(apexA1_, apexA2_ - apexA1_, apexB1_, apexB2_ - apexB1_, hitPos1_, hitPos2_);
				else if((flag&(HIT_AB1 | HIT_BC2)) == (HIT_AB1 | HIT_BC2))
					LineLineProximate(apexA1_, apexA2_ - apexA1_, apexB2_, apexB3_ - apexB2_, hitPos1_, hitPos2_);
				else if((flag&(HIT_AB1 | HIT_CA2)) == (HIT_AB1 | HIT_CA2))
					LineLineProximate(apexA1_, apexA2_ - apexA1_, apexB3_, apexB1_ - apexB3_, hitPos1_, hitPos2_);

				else if((flag&(HIT_BC1 | HIT_AB2)) == (HIT_BC1 | HIT_AB2))
					LineLineProximate(apexA3_, apexA2_ - apexA3_, apexB1_, apexB2_ - apexB1_, hitPos1_, hitPos2_);
				else if((flag&(HIT_BC1 | HIT_BC2)) == (HIT_BC1 | HIT_BC2))
					LineLineProximate(apexA3_, apexA2_ - apexA3_, apexB2_, apexB3_ - apexB2_, hitPos1_, hitPos2_);
				else if((flag&(HIT_BC1 | HIT_CA2)) == (HIT_BC1 | HIT_CA2))
					LineLineProximate(apexA3_, apexA2_ - apexA3_, apexB1_, apexB3_ - apexB1_, hitPos1_, hitPos2_);

				else if((flag&(HIT_CA1 | HIT_AB2)) == (HIT_CA1 | HIT_AB2))
					LineLineProximate(apexA1_, apexA3_ - apexA1_, apexB1_, apexB2_ - apexB1_, hitPos1_, hitPos2_);
				else if((flag&(HIT_CA1 | HIT_BC2)) == (HIT_CA1 | HIT_BC2))
					LineLineProximate(apexA1_, apexA3_ - apexA1_, apexB3_, apexB2_ - apexB3_, hitPos1_, hitPos2_);
				else if((flag&(HIT_CA1 | HIT_CA2)) == (HIT_CA1 | HIT_CA2))
					LineLineProximate(apexA1_, apexA3_ - apexA1_, apexB1_, apexB3_ - apexB1_, hitPos1_, hitPos2_);
				else
					RFASSERT(false, NULL);	
		
				// �H�����ݎZ�o
				*penetrate_ =  *hitPos2_ - *hitPos1_;
				return RF_COLLIDE_EDGE;
			}

			// �Փ˂Ȃ�
			return RF_COLLIDE_NONE;
		}


		/**********************************************************/
		// !!!���̊֐��͍œK�����\���Ǝv����
		// �O�p�|���S���ƎO�p�|���S��
		// �����ŕ�-�ӂ��A���_-�ʂ̏Փ˂����肵�A
		// ��-�ӂ̏ꍇ�͂��ꂼ��̍ŋߐړ_�A
		// ���_-�ʂ̏ꍇ�͒��_��ʂɓ��e�����_��
		// hitPos1_�AhitPos2_�ɏ������ށB
		e_collide_bits TriTriColBothFace(
			const t_vector3&	apexA1_,				// �|���S���̒��_
			const t_vector3&	apexA2_,
			const t_vector3&	apexA3_,
			const t_vector3&	apexB1_,				// �|���S���̒��_
			const t_vector3&	apexB2_,
			const t_vector3&	apexB3_,
			t_vector3*			hitPos1_,				// �Փˈʒu
			t_vector3*			hitPos2_,
			t_vector3*			penetrate_)				// �߂荞�݂̕���// �@��
		{
			// for �O�p�`�ΎO�p�`
			enum HIT_FLAG
			{
				HIT_AB1 = 1,
				HIT_BC1 = 1<<1,
				HIT_CA1 = 1<<2,
				HIT_AB2 = 1<<3,
				HIT_BC2 = 1<<4,
				HIT_CA2 = 1<<5,
			};

			// �ŏ��ɁA�ǂ��Փ˂��邩�𒲂ׂ�
			int hitCount1 = 0;
			int hitFlag1 = 0;
			int hitCount2 = 0;
			int hitFlag2 = 0;

			// �P�̖ʂɑ΂���Q�̕�
			if(TriSegColBothFaces(apexA1_, apexA2_, apexA3_, apexB1_, apexB2_, NULL)==true)
			{
				hitFlag1 |= HIT_AB2;
				hitCount1++;
			}
			if(TriSegColBothFaces(apexA1_, apexA2_, apexA3_, apexB2_, apexB3_, NULL)==true)
			{
				hitFlag1 |= HIT_BC2;
				hitCount1++;
			}
			if(hitCount1 < 2)
			if(TriSegColBothFaces(apexA1_, apexA2_, apexA3_, apexB3_, apexB1_, NULL)==true)
			{
				hitFlag1 |= HIT_CA2;
				hitCount1++;
			}

			// �Q�̖ʂɑ΂���P�̕�
			if(TriSegColBothFaces(apexB1_, apexB2_, apexB3_, apexA1_, apexA2_, NULL)==true)
			{
				hitFlag2 |= HIT_AB1;
				hitCount2++;
			}
			if(TriSegColBothFaces(apexB1_, apexB2_, apexB3_, apexA2_, apexA3_, NULL)==true)
			{
				hitFlag2 |= HIT_BC1;
				hitCount2++;
			}
			if(hitCount2 < 2)
			if(TriSegColBothFaces(apexB1_, apexB2_, apexB3_, apexA3_, apexA1_, NULL)==true)
			{
				hitFlag2 |= HIT_CA1;
				hitCount2++;
			}
	
			// �q�b�g�J�E���g2�͖ʑΒ��_���m��
			// !!!�����ȍ~�ɗ���Ƃ��͒��_�̓����Փ˂��N���Ă��邱�Ƃ��^�����ق�������
			// !!!���_�X�E�B�[�v�����ׂĂ̒��_�ɂ��čs������A
			// ��ԐH������ł����_���Փ˓_�Ƃ���悤�ȏ��������ׂ�
			if(hitCount1 == 2 && hitCount2 == 0)
			{
				float depth = 0;// �x���΍�ɂO�ŏ�����
				t_vector3 n;
				t_vector3 tmp = apexA2_-apexA1_;  // �x���΍�ŏ��������邱�Ƃɂ����̂ŁA���������Ȃ����炱���̂������Ǝv��
				t_vector3 tmp2 = apexA3_-apexA1_; // �x���΍�
				Cross(&n, &tmp, &tmp2);	// �@��
				n = n /D3DXVec3Length(&n);
				switch(hitFlag1)
				{
				case HIT_AB2 | HIT_BC2:
					tmp = apexB2_ - apexA1_;
					depth = Dot(&tmp, &n);
					if(depth > 0)
					{
						tmp = apexB3_ - apexA1_;
						depth = Dot(&tmp, &n);
						*hitPos1_ = *hitPos2_ = apexB3_;
					}
					else
					{
						*hitPos1_ = *hitPos2_ = apexB2_;
					}
					break;
				case HIT_CA2 | HIT_BC2:
					tmp = apexB3_ - apexA1_;
					depth = Dot(&tmp, &n);
					if(depth > 0)
					{
						tmp = apexB1_ - apexA1_;
						depth = Dot(&tmp, &n);
						*hitPos1_ = *hitPos2_ = apexB1_;
					}
					else
					{
						*hitPos1_ = *hitPos2_ = apexB3_;
					}
					break;
				case HIT_AB2 | HIT_CA2:
					tmp = apexB1_ - apexA1_;
					depth = Dot(&tmp, &n);
					if(depth > 0)
					{
						tmp = apexB2_ - apexA1_;
						depth = Dot(&tmp, &n);
						*hitPos1_ = *hitPos2_ = apexB2_;
					}
					else
					{
						*hitPos1_ = *hitPos2_ = apexB1_;
					}
					break;
				default:
					RFASSERT(false, NULL);
				}
				*penetrate_ = n * depth;
				return HRK_COL_VERTEX;
			}

			// �q�b�g�J�E���g2�͖ʑΒ��_���m��
			if(hitCount2 == 2 && hitCount1 == 0)
			{
				float depth = 0;// �x���΍�ɂO�ŏ�����
				t_vector3 n;
				t_vector3 tmp = apexB2_-apexB1_;  // �x���΍�ŏ��������邱�Ƃɂ����̂ŁA���������Ȃ����炱���̂������Ǝv��
				t_vector3 tmp2 = apexB3_-apexB1_; // �x���΍�
				Cross(&n, &tmp, &tmp2);	// �@��
				n = n /D3DXVec3Length(&n);
				switch(hitFlag2)
				{
				case HIT_AB1 | HIT_BC1:
					tmp = apexA2_ - apexB1_;
					depth = Dot(&tmp, &n);
					if(depth > 0)
					{
						tmp = apexA3_ - apexB1_;
						depth = Dot(&tmp, &n);
						*hitPos1_ = *hitPos2_ = apexA3_;
					}
					else
					{
						*hitPos1_ = *hitPos2_ = apexA2_;
					}
					break;
				case HIT_CA1 | HIT_BC1:
					tmp = apexA3_ - apexB1_;
					depth = Dot(&tmp, &n);
					if(depth > 0)
					{
						tmp = apexA1_ - apexB1_;
						depth = Dot(&tmp, &n);
						*hitPos1_ = *hitPos2_ = apexA1_;
					}
					else
					{
						*hitPos1_ = *hitPos2_ = apexA3_;
					}
					break;
				case HIT_AB1 | HIT_CA1:
					tmp = apexA1_ - apexB1_;
					depth = Dot(&tmp, &n);
					if(depth > 0)
					{
						tmp = apexA2_ - apexB1_;
						depth = Dot(&tmp, &n);
						*hitPos1_ = *hitPos2_ = apexA2_;
					}
					else
					{
						*hitPos1_ = *hitPos2_ = apexA1_;
					}
					break;
				default:
					RFASSERT(false, NULL);
				}
				*penetrate_ = n * -depth;

				return HRK_COL_VERTEX;
			}

			// �ӂ̏Փ�
			// ���_-�ӂ��܂߂�
			if(hitCount1 > 0 && hitCount2 > 0)
			{
				int flag = hitFlag2|hitFlag1;
		
				if((flag&(HIT_AB1|HIT_AB2)) == (HIT_AB1|HIT_AB2))
					LineLineProximate(apexA1_, apexA2_ - apexA1_, apexB1_, apexB2_ - apexB1_, hitPos1_, hitPos2_);
				else if((flag&(HIT_AB1 | HIT_BC2)) == (HIT_AB1 | HIT_BC2))
					LineLineProximate(apexA1_, apexA2_ - apexA1_, apexB2_, apexB3_ - apexB2_, hitPos1_, hitPos2_);
				else if((flag&(HIT_AB1 | HIT_CA2)) == (HIT_AB1 | HIT_CA2))
					LineLineProximate(apexA1_, apexA2_ - apexA1_, apexB3_, apexB1_ - apexB3_, hitPos1_, hitPos2_);

				else if((flag&(HIT_BC1 | HIT_AB2)) == (HIT_BC1 | HIT_AB2))
					LineLineProximate(apexA3_, apexA2_ - apexA3_, apexB1_, apexB2_ - apexB1_, hitPos1_, hitPos2_);
				else if((flag&(HIT_BC1 | HIT_BC2)) == (HIT_BC1 | HIT_BC2))
					LineLineProximate(apexA3_, apexA2_ - apexA3_, apexB2_, apexB3_ - apexB2_, hitPos1_, hitPos2_);
				else if((flag&(HIT_BC1 | HIT_CA2)) == (HIT_BC1 | HIT_CA2))
					LineLineProximate(apexA3_, apexA2_ - apexA3_, apexB1_, apexB3_ - apexB1_, hitPos1_, hitPos2_);

				else if((flag&(HIT_CA1 | HIT_AB2)) == (HIT_CA1 | HIT_AB2))
					LineLineProximate(apexA1_, apexA3_ - apexA1_, apexB1_, apexB2_ - apexB1_, hitPos1_, hitPos2_);
				else if((flag&(HIT_CA1 | HIT_BC2)) == (HIT_CA1 | HIT_BC2))
					LineLineProximate(apexA1_, apexA3_ - apexA1_, apexB3_, apexB2_ - apexB3_, hitPos1_, hitPos2_);
				else if((flag&(HIT_CA1 | HIT_CA2)) == (HIT_CA1 | HIT_CA2))
					LineLineProximate(apexA1_, apexA3_ - apexA1_, apexB1_, apexB3_ - apexB1_, hitPos1_, hitPos2_);
				else
					RFASSERT(false, NULL);	
		
				// �H�����ݎZ�o
				*penetrate_ =  *hitPos2_ - *hitPos1_;
				return HRK_COL_LINE;
			}

			// �Փ˂Ȃ�
			return false;
		}


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
			const t_vector3&	apex1_,			
			const t_vector3&	apex2_,
			const t_vector3&	apex3_,
			const t_vector3&	apexD_,
			const t_vector3&	segP_,			
			const t_vector3&	segQ_,
			t_vector3*			intersection_)
		{
			t_vector3 vecPA = apex1_ - segP_;
			t_vector3 vecPB = apex2_ - segP_;
			t_vector3 vecPC = apex3_ - segP_;
			t_vector3 vecPD = apexD_ - segP_;
			t_vector3 vecPQ = segQ_ - segP_;

			// �X�J���O�d�ς̌����@�����QP�~P1���g���܂킷
			t_vector3 cross;
			MakeVectorCross(&cross, vecPQ, vecPC);

			float u = Dot(cross, vecPA);
			if(u < 0)
			{
				// ACB

				float v = Dot(cross, vecPB);
				if(v < 0)
					return false;
				float w = ScalarTriple(vecPQ, vecPB, vecPA);
				if(w < 0)
					return false;

				// �Փˈʒu�͑̐ϔ�ɂ��d�S�ʒu�ɂȂ�
				if(intersection_ != NULL)
				{
					u = -u;
					*intersection_ = (u*vecPB+v*vecPA+w*vecPC)
						/(u+v+w)+segP_;
				}
			}
			else
			{
				// ACD (u>0)���̎O�p�`

				float v = -Dot(cross, vecPD);
				if(v < 0)
					return false;
				float w = ScalarTriple(vecPQ, vecPA, vecPD);
				if(w < 0)
					return false;

				// �Փˈʒu�͑̐ϔ�ɂ��d�S�ʒu�ɂȂ�
				if(intersection_ != NULL)
				{
					*intersection_ = (u*vecPD+v*vecPA+w*vecPC)
						/(u+v+w)+segP_;
				}
			}

			return true; 
		}

		#if 0
			///@brief	AABB-�_�̏Փ�
			///@param	aabb_	AABB
			///@param	pt_		�_�̍��W
			///@retval	true	�Փ˂���
			///@retval	false	�Փ˂Ȃ�
			eSTATUS AABBPtCol(						
				const t_vector3&	centerA_,
				const t_vector3&	halfSizeA_,
				const t_vector3&	point_)
			{
				t_vector3 offset;
				D3DXVec3Subtract(&offset, &point_, &centerA_);

				if (-halfSizeA_.x < offset.x && offset.x < halfSizeA_.x
				&&  -halfSizeA_.y < offset.y && offset.y < halfSizeA_.y
				&&  -halfSizeA_.z < offset.z && offset.z < halfSizeA_.z)
				{
					return true;
				}
				return false;
			}
		#else
			///@brief	AABB-�_�̏Փ�
			///@param	aabb_	AABB
			///@param	pt_		�_�̍��W
			///@retval	true	�Փ˂���
			///@retval	false	�Փ˂Ȃ�
			bool AabbPtCol(
				const CAabb3&	aabb_,
				const t_vector3&			pt_)
			{
				bool retval = (pt_(0) <= aabb_.mini(0) && pt_(0) <= aabb_.mini(0))
					&& (pt_(1) <= aabb_.mini(1) && pt_(1) <= aabb_.mini(1))
					&& (pt_(2) <= aabb_.mini(2) && pt_(2) <= aabb_.mini(2));
				return retval;
			}
		#endif


		///@brief	AABB-�_�ŋߐړ_���擾����
		///@param	aabb_			AABB
		///@param	p_				�_�̍��W
		///@param	proximate_		[out]�ŋߐړ_
		void rf::collision::AabbPtProximate(
			const CAabb3&		aabb_,	
			const t_vector3&	p_,
			t_vector3*			proximate_)
		{
			for (int i=0; i<3; i++)
			{
				if (p_(i) < aabb_.mini(i))
					(*proximate_)(i) = aabb_.mini(i);
				else if (aabb_.maxi(i) < p_(i))
					(*proximate_)(i) = aabb_.maxi(i);
				else
					(*proximate_)(i) = p_(i);		
			}
		}


		///@brief	AABB-���̏Փ˂𔻒肷��
		///@param	aabb_		AABB
		///@param	p_			�_�̍��W
		///@param	radius_		���̔��a
		bool AabbSphereCol(
			const CAabb3&		aabb_,	
			const t_vector3&	p_,
			float				radius_)
		{
			t_vector3 proximate;
			AabbPtProximate(aabb_, p_, &proximate);
			t_vector3 distance(p_ - proximate);
			return distance.LengthSquared() < radius_*radius_;
		}

		///@brief	AABB-�����̏Փ˂𔻒肷��
		///@param	aabb_		AABB
		///@param	rayP_		�����̌��_
		///@param	rayDir_		�����̕����ƒ���
		///@param	normal_		[out]�Փ˖ʂ̖@��
		///@retval	�Փ˓_�܂ł�rayDir_�ɑ΂���W��	
		float AabbRayCol(
			const CAabb3&		aabb_,				
			const t_vector3&	rayP_,					
			const t_vector3&	rayDir_,
			t_vector3*			normal_)
		{
			float distance[3];		// �Փ˕��ʂ܂ł̃p�����[�^����
			float normal[3];		// �Փ˖ʂ��Ƃ̖@���̌��
			bool  inside = true;	// �_��AABB�̓������𔻒肷��t���O

			// �_��AABB�Ɋ܂܂�邩�𔻒肵�A
			// �e���ʂ̖ʂ���̃p�����[�^���������߂�
			for (int i=0; i<3; i++)
			{
				if (rayP_(i) < aabb_.mini(i))
				{
					distance[i] = aabb_.mini(i) - rayP_(i);

					// ���ʂɓ͂��Ȃ��ꍇ
					if (distance[i] > rayDir_(i))
						return RF_BIG_NUM;

					distance[i] /= rayDir_(i);
					normal[i] = -1.0f;
					inside = false;
				}
				else if (rayP_(i) > aabb_.maxi(i))
				{
					distance[i] = aabb_.maxi(i) - rayP_(i);

					// ���ʂɓ͂��Ȃ��ꍇ
					if (distance[i] < rayDir_(i))
						return RF_BIG_NUM;

					distance[i] /= rayDir_(i);
					normal[i] = 1.0f;
					inside = false;
				}
				else
				{
					// �����̏ꍇ��
					distance[i] = -1.0f;
				}
			}

			// �{�b�N�X��������̌����̏ꍇ
			if (inside)
			{
				if (normal_ == NULL)
					return 0.0f;

				*normal_ = -rayDir_;
				normal_->Normalize();
				return 0.0f;
			}

			// �Փ˖ʂ�I������B
			// �_��������Ƃ����������̖ʂ��Փ˖�
			int which = 0;
			float t = 0;
			for (int i=1; i<3; i++)
			{
				if(distance[i] > t)
				{
					which = i;
					t = distance[i];
				}
			}

			// �Փ˖ʂ̂���ȊO�̕����ɂ��͂��Ă��邩���肷��
			for (int i=0; i<3; i++)
			{
				if (which != i)
				{
					float d = rayP_(1) + rayDir_(1)*t;
					if ((d < aabb_.mini(i))
					||  (aabb_.maxi(i) < d))
					{
						return RF_BIG_NUM;
					}
				}
			}

			// �@����Ԃ�
			if (normal_ == NULL)
				return 0.0f;
			for (int i=0; i<3; i++)
			{
				if (i == which)
					(*normal_)(i) = normal[1];
				else
					(*normal_)(i) = 0.0f;
			}
	
			// �p�����[�^��������Ԃ�
			return t;
		}

		///@brief	AABB-���ʂ̏Փ˂𔻒肷��
		///@param	aabb_		AABB
		///@param	planeN_		���ʂ̖@��
		///@param	planeD_		���ʂ̖@�������I�t�Z�b�g
		///@retval	<0	�{�b�N�X�����ʂ̗���
		///@retval	>0	�{�b�N�X�����ʂ̕\��
		///@retval	==0	���ʂƌ�������
		float AabbPlaneCol(
			const CAabb3&		aabb_,				
			const t_vector3&	planeN_,					
			const float			planeD_)
		{
			assert(Dot(planeN_, planeN_)-1.0f < 0.01f);

			// �e�����ɂ��āA�ł��傫���E�������l����ݍ���
			float dMin = 0;
			float dMax = 0;
			for (int i=0; i<3; i++)
			{
				if (planeN_(i) > 0.0f)
				{
					dMin += planeN_(i)*aabb_.mini(i);
					dMax += planeN_(i)*aabb_.maxi(i);
				}
				else
				{
					dMin += planeN_(i)*aabb_.maxi(i);
					dMax += planeN_(i)*aabb_.mini(i);
				}
			}

			// ���ʂ̖@�������I�t�Z�b�g�ƁAmini�Emaxi�̔�r������
			if (dMin >= planeD_)
			{
				return dMin - planeD_;
				// return 1;
			}
			else if (dMax <= planeD_)
			{
				return dMax - planeD_;
				// return -1;
			}

			return 0;
		}


		///@brief	AABB-�ړ����ʂ̏Փ˂𔻒肷��
		///@param	aabb_		AABB
		///@param	relAabbV_	���ʂɑ΂���AABB�̑��Α��x
		///@param	planeN_		���ʂ̖@��
		///@param	planeD_		���ʂ̖@�������I�t�Z�b�g
		///@retval	�Փ˓_�܂ł�relAabbV_�ɑ΂���W��	
		float AabbPlaneCol(
			const CAabb3&		aabb_,			
			const t_vector3&	relAabbV_,				
			const t_vector3&	planeN_,					
			const float&		planeD_)
		{
			assert(Dot(relAabbV_, relAabbV_)-1.0f < 0.01f);
			assert(Dot(planeN_, planeN_)-1.0f < 0.01f);

			// AABB�̈ړ����������ʂ̕\�Ɍ������Ă��邩���m�F����
			float dot = Dot(planeN_, relAabbV_);
			if (dot >= 0.0f)
			{
				return RF_BIG_NUM;
			}

			// �@���ɂ��āA�ł��傫���E���������ς��Z�o����
			// ���̂悤�ȓ_���̂����ʂ��Ȃ��Ă�
			// xyz�e�����ɂ��čŏ��E�ő�̏�ݍ��݂Œl�͋��܂�
			float dMin = 0;
			float dMax = 0;
			for (int i=0; i<3; i++)
			{
				if (planeN_(i) > 0.0f)
				{
					dMin += planeN_(i)*aabb_.mini(i);
					dMax += planeN_(i)*aabb_.maxi(i);
				}
				else
				{
					dMin += planeN_(i)*aabb_.maxi(i);
					dMax += planeN_(i)*aabb_.mini(i);
				}
			}

			// �ő�̓��ς����ʂ̃I�t�Z�b�g��菬�����Ȃ�A
			// AABB�͕��ʂ̗����ɂ���B
			if (dMax <= planeD_)
			{
				return RF_BIG_NUM;
			}

			// AABB���̍ŋߐړ_��dMin
			// ���̓_��AABB�̑��x�������������Ƃ���
			// �����ƕ��ʂ̌�_�����߂�
			float t = (planeD_ - dMin) / dot;

			// ���߂���������Ă���
			if (t < 0.0f)
			{
				return 0;
			}

			// �Փ˓_��Ԃ�
			return t;
		}

		#if 1
			///@brief	AABB-AABB�̏Փ˂𔻒肷��
			///@param	aabbA_	AABB_A
			///@param	aabbB_	AABB_B
			///@retval	true	�Փ˂���
			///@retval	false	�Փ˂Ȃ�
			///@note	center-size�^��AABB�Ȃ�A�傫��AABB�Ɠ_��
			/// 			�ˉe���ďՓ˂𔻒肵�������������B
			/// 			mini-maxi�^��AABB�Ȃ炻�̌���ł͂Ȃ��H
			bool AabbAabbCol(
				const CAabb3&	aabbA_,
				const CAabb3&	aabbB_)
			{
				for (int i=0; i<3; i++)
				{
					if (aabbA_.mini(i) > aabbB_.maxi(i))
						return false;
					if (aabbA_.maxi(i) > aabbB_.mini(i))
						return false;
				}
				return true;
			}
		#else
			// AABB-AABB �傫��AABB�Ɠ_�Ɏˉe���Čv�Z
			// Axis Aligned Bounding Box �傫��AABB�Ɠ_�Ɏˉe���Čv�Z
			bool AabbAabbCol(
				const t_vector3&	centerA_,
				const t_vector3&	halfSizeA_,
				const t_vector3&	centerB_,
				const t_vector3&	halfSizeB_)
			{
				t_vector3 size;
				t_vector3 offset;
		
				D3DXVec3Add(&size, &halfSizeA_, &halfSizeB_);
				D3DXVec3Subtract(&offset, &centerB_, &centerA_);

				if (-size.x < offset.x && offset.x < size.x
				&&  -size.y < offset.y && offset.y < size.y
				&&  -size.z < offset.z && offset.z < size.z)
				{
					return true;
				}

				return false;
			}
		#endif

		///@brief	AABB-AABB�̓��I�Փ˓_�����߂�B
		///@note	AABB_A��ɂ��āAAABB_A�͐Î~���Ă���Ƃ��Čv�Z����
		///@param	aabbA_		AABB_A
		///@param	aabbB_		AABB_B
		///@param	relAabbBV_	A�ɑ΂���B�̑��Α��x
		///@retval	�����J�n����relAabbBV_�ɑ΂���p�����[�^������
		float AabbAabbCol(
			const CAabb3&		aabbA_,
			const CAabb3&		aabbB_,
			const t_vector3&	relVelocityB_)
		{
			// �S����enter��leave�A�e���̐ϏW���ɂȂ�
			float enterAll = 0.0f;
			float leaveAll = 1.0f;

			for (int i=0; i<3; i++)
			{
				if (relVelocityB_(i) == 0.0f)
				{
					// i�Ԗڂ̎������̑��Α��x���Ȃ��������Î~�Ɠ���

					// �͂��߂���Փ˂��Ă��Ȃ��Ȃ�A�ȍ~���Փ˂͂Ȃ�
					if ((aabbB_.maxi(i) <= aabbA_.mini(i)) 
					||  (aabbA_.maxi(i) <= aabbB_.mini(i))) 
					{
						return RF_BIG_NUM;
					}
				}
				else
				{
					// ���Α��x������ꍇ
			
					float overV = 1.0f / relVelocityB_(i); 
					float enter = aabbA_.mini(i) - aabbB_.maxi(i) * overV;
					float leave = aabbA_.maxi(i) - aabbB_.mini(i) * overV;

					// �i�s�����ɂ���Ă�enter>leave�ƂȂ�ꍇ�̃X���b�v
					if (enter > leave)
					{
						(std::swap)(enter, leave);
					}

					// �S���̃C���^�[�o�����X�V����
					if (enter > enterAll)
						enterAll = enter;
					if (leave < leaveAll)
						leaveAll = leave;

					// �C���^�[�o������B�������Ȃ�
					// ���ڂ͕K�v�Ȃ��͂�
					if (enterAll > leaveAll)
					{
						return RF_BIG_NUM;
					}
				}
			}

			// ��������
			return enterAll;
		}


		#if 0
			/**********************************************************/
			// ���r���[��OBB�|OBB�𔻒肷��̂�
			// �����y���B���ꍇ�ɂ���Ă͋t����
			// ���E�{�����[����OBB���g���悤�ȂƂ��͑������ɗ�����
			eSTATUS OBBOBBColAbbr(
				const t_vector3&	sizeA_,				// ���S����ʂւ̋���
				const t_vector3&	sizeB_,
				const t_matrix4&	worldA_,			// ���ꂼ��̈ʒu�p��
				const t_matrix4&	worldB_
			)
			{
				t_matrix4 T;		// �ˉe���ꂽ�����ϊ��s��
				t_matrix4 absT;	// �ˉe���ꂽ��̒��g�̐�Βl���Ƃ�������
				t_vector3 vecAB;
				float projLenA;

				for(int i = 0; i<3; i++)	// ���g�̃��[�J����ԂɎˉe���t�s��i���̏ꍇ�]�u�j���|����
				for(int j = 0; j<3; j++)
				{
					T(i,j) = worldB_(0,j) * worldA_(0,i)	// ���ʓI�ɓ]�n�s��Ƃ̐ςɂȂ遨�ˉe���ł���
						   + worldB_(1,j) * worldA_(1,i)
						   + worldB_(2,j) * worldA_(2,i);
				}
				for(int i = 0; i<3; i++)	// �{���͌v�Z���ōs����ׂ�
				for(int j = 0; j<3; j++)
					absT(i,j) = fabs(T(i,j)); 
		
				vecAB = t_vector3(	// �{�b�N�X�̒��S�ԃx�N�g��dest-scr�ɂ��Ă���E�E�E
					worldB_._41 - worldA_._41,
					worldB_._42 - worldA_._42,
					worldB_._43 - worldA_._43);

				vecAB = t_vector3(	// ���S�ԃx�N�g��/���������Ɏˉe
					vecAB.x*worldA_._11 + vecAB.y*worldA_._21 + vecAB.z*worldA_._31,
					vecAB.x*worldA_._12 + vecAB.y*worldA_._22 + vecAB.z*worldA_._32,
					vecAB.x*worldA_._13 + vecAB.y*worldA_._23 + vecAB.z*worldA_._33);

				// �e�ʂɐ����ȕ������ɂ�锻��
				projLenA = absT._11 * sizeB_.x			// ����OBB���W�n��x�������ˉe�Ŕ���
						 + absT._12 * sizeB_.y
						 + absT._13 * sizeB_.z;
				if(fabs(vecAB.x) > sizeA_.x + projLenA)
					return false;

				projLenA = absT._21 * sizeB_.x			// ����OBB���W�n��y�������ˉe�Ŕ���
						 + absT._22 * sizeB_.y
						 + absT._23 * sizeB_.z;
				if(fabs(vecAB.y) > sizeA_.y + projLenA)
					return false;

				projLenA = absT._31 * sizeB_.x			// ����OBB���W�n��z�������ˉe�Ŕ���
						 + absT._32 * sizeB_.y
						 + absT._33 * sizeB_.z;
				if(fabs(vecAB.z) > sizeA_.z + projLenA)
					return false;

				projLenA = absT._11 * sizeA_.x			// ��������OBB���W�n��x�������ˉe�Ŕ���
						 + absT._12 * sizeA_.y
						 + absT._13 * sizeA_.z;
				if(fabs(vecAB.x*T._11 + vecAB.y*T._21 + vecAB.z*T._31) > sizeB_.x + projLenA)
					return false;

				projLenA = absT._21 * sizeA_.x			// ��������OBB���W�n��y�������ˉe�Ŕ���
						 + absT._22 * sizeA_.y
						 + absT._23 * sizeA_.z;
				if(fabs(vecAB.x*T._12 + vecAB.y*T._22 + vecAB.z*T._32) > sizeB_.y + projLenA)
					return false;

				projLenA = absT._31 * sizeA_.x			// ��������OBB���W�n��z�������ˉe�Ŕ���
						 + absT._32 * sizeA_.y
						 + absT._33 * sizeA_.z;
				if(fabs(vecAB.x*T._13 + vecAB.y*T._23 + vecAB.z*T._33) > sizeB_.z + projLenA)
					return false;

				return true;
			}

			/**********************************************************/
			eSTATUS OBBCol(
				const t_vector3&	sizeA_,				// ���S����ʂւ̋���
				const t_vector3&	sizeB_,
				const t_matrix4&	worldA_,			// ���ꂼ��̈ʒu�p��
				const t_matrix4&	worldB_
			)
			{

				t_matrix4 T;		// �ˉe���ꂽ�����ϊ��s��
				t_matrix4 absT;	// �ˉe���ꂽ��̒��g�̐�Βl���Ƃ�������
				t_vector3 vecAB;

				float projLenA;
				float projLenB;

				for(int i = 0; i<3; i++)	// ���g�̃��[�J����ԂɎˉe���t�s��i���̏ꍇ�]�u�j���|����
				for(int j = 0; j<3; j++)
				{
					T(i,j) = worldB_(0,j) * worldA_(0,i)	// ���ʓI�ɓ]�n�s��Ƃ̐ςɂȂ遨�ˉe���ł���
						   + worldB_(1,j) * worldA_(1,i)
						   + worldB_(2,j) * worldA_(2,i);
				}
		
				for(int i = 0; i<3; i++)	// �{���͌v�Z���ōs����ׂ�
				for(int j = 0; j<3; j++)
					absT(i,j) = fabs(T(i,j)); 
		
				vecAB = t_vector3(		// �{�b�N�X�̒��S�ԃx�N�g��dest-scr�ɂ��Ă���E�E�E
					worldB_._41 - worldA_._41,
					worldB_._42 - worldA_._42,
					worldB_._43 - worldA_._43);

				vecAB = t_vector3(		// ���S�ԃx�N�g��/���������Ɏˉe
					vecAB.x*worldA_._11 + vecAB.y*worldA_._21 + vecAB.z*worldA_._31,
					vecAB.x*worldA_._12 + vecAB.y*worldA_._22 + vecAB.z*worldA_._32,
					vecAB.x*worldA_._13 + vecAB.y*worldA_._23 + vecAB.z*worldA_._33);

				// �������n�܂�܂�^^;
		
				projLenA = absT._11 * sizeB_.x			// ����OBB���W�n��x�������ˉe�Ŕ���
						 + absT._12 * sizeB_.y
						 + absT._13 * sizeB_.z;
				if(fabs(vecAB.x) > sizeA_.x + projLenA)
					return false;

				projLenA = absT._21 * sizeB_.x			// ����OBB���W�n��y�������ˉe�Ŕ���
						 + absT._22 * sizeB_.y
						 + absT._23 * sizeB_.z;
				if(fabs(vecAB.y) > sizeA_.y + projLenA)
					return false;

				projLenA = absT._31 * sizeB_.x			// ����OBB���W�n��z�������ˉe�Ŕ���
						 + absT._32 * sizeB_.y
						 + absT._33 * sizeB_.z;
				if(fabs(vecAB.z) > sizeA_.z + projLenA)
					return false;

				projLenA = absT._11 * sizeA_.x			// ��������OBB���W�n��x�������ˉe�Ŕ���
						 + absT._12 * sizeA_.y
						 + absT._13 * sizeA_.z;
				if(fabs(vecAB.x*T._11 + vecAB.y*T._21 + vecAB.z*T._31) > sizeB_.x + projLenA)
					return false;

				projLenA = absT._21 * sizeA_.x			// ��������OBB���W�n��y�������ˉe�Ŕ���
						 + absT._22 * sizeA_.y
						 + absT._23 * sizeA_.z;
				if(fabs(vecAB.x*T._12 + vecAB.y*T._22 + vecAB.z*T._32) > sizeB_.y + projLenA)
					return false;

				projLenA = absT._31 * sizeA_.x			// ��������OBB���W�n��z�������ˉe�Ŕ���
						 + absT._32 * sizeA_.y
						 + absT._33 * sizeA_.z;
				if(fabs(vecAB.x*T._13 + vecAB.y*T._23 + vecAB.z*T._33) > sizeB_.z + projLenA)
					return false;

				projLenA = absT._13 * sizeA_.y			// x���~x���Ɏˉe�Ŕ���
					+ absT._12 * sizeA_.z;
				projLenB = absT._31 * sizeB_.y
					+ absT._21 * sizeB_.z;
				if(fabs(vecAB.z*T._12 - vecAB.y*T._13) > projLenA + projLenB)
					return false;

				projLenA = absT._23 * sizeA_.y			// x���~y���Ɏˉe�Ŕ���
					+ absT._22 * sizeA_.z;
				projLenB = absT._31 * sizeA_.x
					+ absT._11 * sizeB_.z;
				if(fabs(vecAB.z*T._22 - vecAB.y*T._23) > projLenA + projLenB)
					return false;

				projLenA = absT._33 * sizeA_.y			// x���~z���Ɏˉe�Ŕ���
					+ absT._32 * sizeA_.z;
				projLenB = absT._21 * sizeB_.x
					+ absT._11 * sizeB_.y;
				if(fabs(vecAB.z*T._32 - vecAB.y*T._33) > projLenA + projLenB)
					return false;

				projLenA = absT._13 * sizeA_.x			// y���~x���Ɏˉe�Ŕ���
					+ absT._11 * sizeA_.z;
				projLenB = absT._32 * sizeB_.y
					+ absT._22 * sizeB_.z;
				if(fabs(vecAB.x*T._13 - vecAB.z*T._11) > projLenA + projLenB)
					return false;

				projLenA = absT._23 * sizeA_.x			// y���~y���Ɏˉe�Ŕ���
					+ absT._21 * sizeA_.z;
				projLenB = absT._32 * sizeB_.x
					+ absT._12 * sizeB_.z;
				if(fabs(vecAB.x*T._23 - vecAB.z*T._21) > projLenA + projLenB)
					return false;

				projLenA = absT._33 * sizeA_.x			// y���~z���Ɏˉe�Ŕ���
					+ absT._31 * sizeA_.z;
				projLenB = absT._22 * sizeB_.x
					+ absT._12 * sizeB_.y;
				if(fabs(vecAB.x*T._33 - vecAB.z*T._31) > projLenA + projLenB)
					return false;

				projLenA = absT._12 * sizeA_.x			// z���~x���Ɏˉe�Ŕ���
					+ absT._11 * sizeA_.y;
				projLenB = absT._33 * sizeB_.y
					+ absT._23 * sizeB_.z;
				if(fabs(vecAB.y*T._11 - vecAB.x*T._12) > projLenA + projLenB)
					return false;

				projLenA = absT._22 * sizeA_.x			// z���~y���Ɏˉe�Ŕ���
					+ absT._21 * sizeA_.y;
				projLenB = absT._33 * sizeB_.x
					+ absT._13 * sizeB_.z;
				if(fabs(vecAB.y*T._21 - vecAB.x*T._22) > projLenA + projLenB)
					return false;

				projLenA = absT._32 * sizeA_.x			// z���~z���Ɏˉe�Ŕ���
					+ absT._31 * sizeA_.y;
				projLenB = absT._23 * sizeB_.x
					+ absT._13 * sizeB_.y;
				if(fabs(vecAB.y*T._31 - vecAB.x*T._32) > projLenA + projLenB)
					return false;

				// �Փ˂��Ă���
				return true;
			}
			/**********************************************************/
			// �ʃ��b�V���Ƃ��̃��[�J���ł̓_
			// ��ԋ߂��ʂւ̓��e�Ƃ��̕����̐H�����݂�Ԃ�
			eSTATUS MeshPtCol(
				const CTriMesh&		mesh_,
				const t_vector3&	p_,			// �Փˈʒu�i���b�V���̃��[�J���j
				float*			depth_, 
				t_vector3*		n_)
			{
				t_vector3* Tri[3];
				t_vector3 N;
				float depth;

				// �Ƃ肠����������
				*depth_ = -10000;

				for(int i=0; i < mesh_.m_numIB; i+=3)	// A�̖ʂɑ΂���
				{
					Tri[0] = &mesh_.m_VB[mesh_.m_IB[i]];		// ��
					Tri[1] = &mesh_.m_VB[mesh_.m_IB[i+1]];
					Tri[2] = &mesh_.m_VB[mesh_.m_IB[i+2]];
			
					if(PlanePt(*Tri[0], *Tri[1], *Tri[2], p_, &depth, &N) == false)
						return false;
			
					if(depth > *depth_)
					{
						*depth_ = depth;
						*n_ = N;
					}
				}
				// ��̃��[�v��S�������Ȃ璸�_�Փ�
				return true;
			}


			/**********************************************************/
			// ���b�V���Ɛ����Փ˔���Ж�
			// ��ԋ߂��ʂւ̓��e�Ƃ��̕����̐H�����݂�Ԃ�
			eSTATUS MeshSegCol(
				const CTriMesh&		mesh_,
				const t_vector3&	segP_,					// �����̎n�_
				const t_vector3&	segDir_,				// �����̕���// �ƒ����H��������
				float*			distance_)
			{
				t_vector3* Tri[3];
				t_vector3 penetrate;
				t_vector3 N;

				// �Ƃ肠����������
				*distance_ = -HRK_FLOAT_MAX;


				// ��ԏ��߂Ɍ����������T�ŏ������f����
				// !!!��d�Ɍ��������ꍇ�ɁA�߂��ق��������ق����킩��Ȃ����ǁA�A�������H
				for(int i=0; i < mesh_.m_numIB; i+=3)	// A�̖ʂɑ΂���
				{
					Tri[0] = &(mesh_.m_VB[mesh_.m_IB[i]]);		// ��
					Tri[1] = &(mesh_.m_VB[mesh_.m_IB[i+1]]);
					Tri[2] = &(mesh_.m_VB[mesh_.m_IB[i+2]]);
			
					if(TriSegCol(*Tri[0], *Tri[1], *Tri[2], segP_, segDir_, distance_) == true)
					{
						return true;
					}
					// �L���ɂ���Ƃ����Ɠ���
					// if(depth > *depth_)
					// {
					// 	*depth_ = depth;
					// 	*n_ = N;
					// }
				}
				// ��̃��[�v��S�������Ȃ�q�b�g�Ȃ�
				return false;

			}
			/**********************************************************/
			// ���ꂼ��̃��[�J���ł̏Փˈʒu���o
			// ���񂩍s�����Ƃŕ��̂����S�ɕ�������
			// colPosA_�FA�̏Փˈʒu�iA���[�J���j
			// colPosB_�FB�̏Փˈʒu�iB���[�J���j
			// penetrate_�F�Փ˕����Ɛ[��(��΍��W)
			eSTATUS MeshColPos(
				const CTriMesh&		meshA_,
				const CTriMesh&		meshB_,
				const t_matrix4&	worldA_,				// ���ꂼ��̈ʒu�p��
				const t_matrix4&	worldB_,
				t_vector3*		colPosA_,		// A�̏Փˈʒu�iA���[�J���j
				t_vector3*		colPosB_,		// B�̏Փˈʒu�iB���[�J���j
				t_vector3*		penetrate_)		// �Փ˕����Ɛ[��(��΍��W)
			{
				// �_-���b�V���̔���
				t_vector3 n;
				float	depth;
				eSTATUS ret = false;
				t_vector3 hitA(0,0,0);	// �Փˈʒu
				t_vector3 hitB(0,0,0);
				t_vector3 tmpV1;
				t_vector3 tmpV2;
				float cnt = 0;

				// 081205�����ł͂��ׂĂ̖ʂ̕\���ɓ_�����邩���`�F�b�N�B
				// A��̓_����B�̍��W�n��
				t_matrix4 T;
				t_matrix4 tmpM;
				D3DXMatrixInverse(&tmpM, NULL, &worldB_);
				D3DXMatrixMultiply(&T, &worldA_, &tmpM);
				for(int i=0; i < meshA_.m_numVB; i++)	// A�̒��_
				{
					D3DXVec3TransformCoord(&tmpV1, &meshA_.m_VB[i], &T);		// B���[�J���ړ�
					if(MeshPtCol(meshB_, tmpV1, &depth, &n) == true)
					{
						cnt++;
						n *= -depth;
						tmpV1 = tmpV1 - n;
						hitB += tmpV1;
						D3DXMatrixInverse(&tmpM, NULL, &T);
						D3DXVec3TransformCoord(&tmpV2, &tmpV1, &tmpM);
						hitA += tmpV2;
						D3DXVec3TransformNormal(penetrate_, &n, &worldB_);
						ret = HRK_COL_VERTEX;
					}
				}
		
				// B��̓_����A�̃��[�J����
				D3DXMatrixInverse(&tmpM, NULL, &worldA_);
				D3DXMatrixMultiply(&T, &worldB_, &tmpM);
				for(int i=0; i < meshB_.m_numVB; i++)	// B�̒��_
				{
					D3DXVec3TransformCoord(&tmpV1, &meshB_.m_VB[i], &T);// ���_
					if(MeshPtCol(meshA_, tmpV1, &depth, &n) == true)
					{
						cnt++;
						n *= depth;
						tmpV1 = tmpV1 - n;
						hitA += tmpV1;
						D3DXMatrixInverse(&tmpM, NULL, &T);
						D3DXVec3TransformCoord(&tmpV2, &tmpV1, &tmpM);
						hitB += tmpV2;
						D3DXVec3TransformNormal(penetrate_, &n, &worldA_);
						ret = HRK_COL_VERTEX;
					}
				}
				if(ret != false)
				{
					*colPosA_ = hitA/cnt;
					*colPosB_ = hitB/cnt;
					return ret;
				}

				// ��-�ӂ̏Փ�	
				t_vector3 TriA[3];
				t_vector3 TriB[3];

				t_vector3 penetrate;
				eSTATUS	result;			// �Փˌ��ʂ̈ꎞ�I�ȕۑ�
				float minPenetrate = 10000;
				float maxPenetrate = 0;
				float len = 0;
				for(int i = 0; i < meshA_.m_numIB; i+=3)	
				for(int j = 0; j < meshB_.m_numIB; j+=3)		
				{
					// ���W�ϊ�
					TriA[0] = meshA_.m_VB[meshA_.m_IB[i]];
					TriA[1] = meshA_.m_VB[meshA_.m_IB[i+1]];
					TriA[2] = meshA_.m_VB[meshA_.m_IB[i+2]];
					D3DXVec3TransformCoord(&TriB[0], &meshB_.m_VB[meshB_.m_IB[j]], &T);
					D3DXVec3TransformCoord(&TriB[1], &meshB_.m_VB[meshB_.m_IB[j+1]], &T);
					D3DXVec3TransformCoord(&TriB[2], &meshB_.m_VB[meshB_.m_IB[j+2]], &T);
			
					// ���_�̈ʒu�֌W����Փˉ\���̂���O�p�`�̑g�ݍ��킹�����`�F�b�N
					// �����Փ˂������Ȃ猵���Ƀ`�F�b�N����
					// !!!���̏����͋t�ɏd���Ȃ�\�������邽�߁A���x���؂���������
					if(TriTriCoPosibility(
						TriA[0], TriA[1], TriA[2],
						TriB[0], TriB[1], TriB[2]
						) == false)
					{
						// �Փˌ��o
						result = TriTriCollidePos(
							TriA[0], TriA[1], TriA[2],
							TriB[0], TriB[1], TriB[2],
							&tmpV1, &tmpV2, &penetrate);
						if(result == HRK_COL_LINE && result != HRK_COL_VERTEX)
						// if(result != false)
						{		
							len = D3DXVec3Length(&penetrate);
							if(minPenetrate > len)				// �ł��u�󂢁v�H�����݂�������
							{	
								minPenetrate = len;
								*colPosA_ = tmpV1;				// A�Փ˓_A�̃��[�J��
								D3DXMatrixInverse(&tmpM, NULL, &T);
								D3DXVec3TransformCoord(colPosB_,&tmpV2, &tmpM);// B�Փ˓_B�̃��[�J��
								D3DXVec3TransformNormal(penetrate_, &penetrate, &worldA_);	// ��΍��W�n
								ret = result;
							}
						}
						if(result == HRK_COL_VERTEX)
						{		
							len = D3DXVec3Length(&penetrate);
							if(maxPenetrate < len)				// �ł��u�󂢁v�H�����݂�������
							{	
								cnt++;
								maxPenetrate = len;
								hitA += tmpV1;				// A�Փ˓_A�̃��[�J��
								D3DXMatrixInverse(&tmpM, NULL, &T);
								D3DXVec3TransformCoord(&tmpV2,&tmpV1, &tmpM);// B�Փ˓_B�̃��[�J��
								hitB += tmpV2;
								D3DXVec3TransformNormal(penetrate_, &penetrate, &worldA_);	// ��΍��W�n
								ret = result;
							}
						}
						// assert(result == HRK_COL_VERTEX);
 					}
				}
				if(ret == HRK_COL_VERTEX)
				{
					*colPosA_ = hitA/cnt;
					*colPosB_ = hitB/cnt;
				}
				return ret;
			}

			/**********************************************************
			eSTATUS CClollisionFunc3D::MeshColPos(
				const CTriMesh&		meshA_,
				const CTriMesh&		meshB_,
				const t_matrix4&	worldA_,				// ���ꂼ��̈ʒu�p��
				const t_matrix4&	worldB_,
				const t_vector3&	vA_,					// ��]���S�̈ʒu
				const t_vector3&	vB_,
				const t_vector3&	omegaA_,				// ���[�J���ł̊p���x
				const t_vector3&	omegaB_,
				t_vector3*		colPosA_,		// A�̏Փˈʒu�iA���̒��S����̑��Έʒu�E��΍��W�j
				t_vector3*		colPosB_,		// B�̏Փˈʒu�iB���̒��S����̑��Έʒu�E��΍��W�j
				t_vector3*		penetrate_)		// �Փ˕����Ɛ[��
			{

				// ���΍��W�ɕϊ���B����A�̃��[�J����
				t_matrix4 T;
				t_matrix4 tmp;
				D3DXMatrixInverse(&tmp, NULL, &worldA_);
				D3DXMatrixMultiply(&T, &worldB_, &tmp);

				//// �_�̔���
				// int cnt == 0;
				// t_vector3 TriA[3];
				// t_vector3 TriB[3];
				// t_vector3 hitbuf[8];	// �o�b�t�@���K��
				// t_vector3 hit;
				// for(int j = 0; j < meshB_.m_numVB; j++)	// B�̒��_
				// {
				// 	for(int i = 0; i < meshA_.m_numIB; i+=3)	// A�̖ʂɑ΂���
				// 	{
				// 		TriA[0] = meshA_.m_VB[meshA_.m_IB[i]];		// ��
				// 		TriA[1] = meshA_.m_VB[meshA_.m_IB[i+1]];
				// 		TriA[2] = meshA_.m_VB[meshA_.m_IB[i+2]];
				// 		D3DXVec3TransformCoord(&TriB[0], &meshB_.m_VB[j], &T);// ���_
				// 		if(PlanePt(TriA[0], TriA[1], TriA[2], TriB[0])==false)
				// 			break;
				// 	}
				// 	if(meshA_.m_numIB == i)	// ��̃��[�v��S������
				// 	{
				// 		hitbuf[cnt] = TriB[0];
				// 		cnt++;
				// 	}
				// }
				// if(cnt > 0)
				// {
				// 	// �Փˉӏ��̕���
				// 	for(int i = 0; i < cnt;i++)
				// 	{
				// 		hit += hitbuf[i]
				// 	}
				// 	hit /= cnt;

				// }








				// for(int j = 0; j < meshA_.m_numVB; j++)	// A�̒��_
				// {
				// 	for(int i = 0; i < meshB_.m_numIB; i+=3)	// B�̖ʂɑ΂���
				// 	{
				// 		TriB[0] = meshB_.m_VB[meshB_.m_IB[i]];		// ��
				// 		TriB[1] = meshB_.m_VB[meshB_.m_IB[i+1]];
				// 		TriB[2] = meshB_.m_VB[meshB_.m_IB[i+2]];
				// 		D3DXVec3TransformCoord(&TriA[0], &meshA_.m_VB[j], &T);// ���_
				// 		if(PlanePt(TriB[0], TriB[1], TriB[2], TriA[0])==false)
				// 			break;
				// 	}
				// 	if(meshA_.m_numIB == i)	// ��̃��[�v��S������
				// 	{
				// 		hitbuf[cnt] = TriB[0];
				// 		cnt++;
				// 	}
				// }



















			t_vector3 TriA[3];
			t_vector3 TriB[3];
			// A�̃|���S���ɑ΂���B�̃|���S���̌����\���`�F�b�N
			// ���������Ēx���Ȃ邩��
			float maxPenetrate = 0;
			float minPenetrate = 10000;
			float penetrateLen = 0;
			t_vector3 hitA;
			t_vector3 hitB;
			t_vector3 penetrate;
			t_vector3 penetrateV;
			t_vector3 penetrateL;
			eSTATUS ret = false;
			for(int i = 0; i < meshA_.m_numIB - 3; i+=3)	
			for(int j = 0; j < meshB_.m_numIB - 3; j+=3)		
			{
				// ���W�ϊ�
				TriA[0] = meshA_.m_VB[meshA_.m_IB[i]];
				TriA[1] = meshA_.m_VB[meshA_.m_IB[i+1]];
				TriA[2] = meshA_.m_VB[meshA_.m_IB[i+2]];
				D3DXVec3TransformCoord(&TriB[0], &meshB_.m_VB[meshB_.m_IB[j]], &T);
				D3DXVec3TransformCoord(&TriB[1], &meshB_.m_VB[meshB_.m_IB[j+1]], &T);
				D3DXVec3TransformCoord(&TriB[2], &meshB_.m_VB[meshB_.m_IB[j+2]], &T);
				// �����Փ˂������Ȃ猵���Ƀ`�F�b�N����
				// if(TriTriCoPosibility(
				// 	TriA[0],
				// 	TriA[1],
				// 	TriA[2],
				// 	TriB[0],
				// 	TriB[1],
				// 	TriB[2]
				// 	) == false)
				// {
					// ����Ɏg�����O�p�`���m���Փ˂̉\���������Ă���B
					// �Փˈʒu
					eSTATUS result;
					result = TriTriCollidePos(
						TriA[0], TriA[1], TriA[2],
						TriB[0], TriB[1], TriB[2],
						&hitA, &hitB, &penetrate);
					if(result == HRK_RET_2)// ��ɕӏՓ�
					{		
						penetrateLen = D3DXVec3Length(&penetrate);
						if(minPenetrate > penetrateLen)
						{	
							result = TriTriCollidePos(
								TriA[0], TriA[1], TriA[2],
								TriB[0], TriB[1], TriB[2],
								&hitA, &hitB, &penetrate);
							*penetrate_ = penetrate; 
							minPenetrate = penetrateLen;
							*colPosA_ = hitA;
							D3DXMatrixInverse(&tmp, NULL, &T);
							D3DXVec3TransformCoord(colPosB_,&hitA, &tmp);
						}
						ret = HRK_RET_2;
					}
					// if(result == true && ret != HRK_RET_2)	// ���_�Փ�
					// {				
					// 	penetrateLen = D3DXVec3Length(&penetrate);
					// 	if(maxPenetrate < penetrateLen || ret == HRK_RET_2)
					// 	{
					// 		*penetrate_ = penetrate; 
					// 		maxPenetrate = penetrateLen;
					// 		*colPosA_ = hitA;
					// 		D3DXMatrixInverse(&tmp, NULL, &T);
					// 		D3DXVec3TransformCoord(colPosB_,&hitA, &tmp);
					// 	}
					// 	ret = true;
					// }

 				}
			}

			return ret;

		}
		/**********************************************************
		eSTATUS CClollisionFunc3D::MeshColPos(
			const CTriMesh&	meshA_,
			const CTriMesh&	meshB_,
			const t_matrix4&	worldA_,				// ���ꂼ��̈ʒu�p��
			const t_matrix4&	worldB_,
			const t_vector3&	vA_,					// ��]���S�̈ʒu
			const t_vector3&	vB_,
			const t_vector3&	omegaA_,				// ���[�J���ł̊p���x
			const t_vector3&	omegaB_,
			t_vector3*		colPosA_,		// A�̏Փˈʒu�iA���̒��S����̑��Έʒu�E��΍��W�j
			t_vector3*		colPosB_,		// B�̏Փˈʒu�iB���̒��S����̑��Έʒu�E��΍��W�j
			t_vector3*		penetrate_)		// �Փ˕����Ɛ[��
		{

			// ���΍��W�ɕϊ���B����A�̃��[�J����
			t_matrix4 T;
			t_matrix4 tmp;
			D3DXMatrixInverse(&tmp, NULL, &worldA_);
			D3DXMatrixMultiply(&T, &worldB_, &tmp);

			// ���Α��x�E�p���x
			t_vector3 v = vB_-vA_;
			t_vector3 omega = omegaB_ - omegaA_;

			// ���_�̃X�E�B�[�v
			t_vector3 vertexSweep;

			t_vector3 TriA[3];
			t_vector3 TriB[3];
			// A�̃|���S���ɑ΂���B�̃|���S���̌����\���`�F�b�N
			// ���������Ēx���Ȃ邩��
			for(int i = 0; i < meshA_.m_numIB - 3; i+=3)	
			for(int j = 0; j < meshB_.m_numIB - 3; j+=3)		
			{
				// ���W�ϊ�
				TriA[0] = meshA_.m_VB[meshA_.m_IB[i]];
				TriA[1] = meshA_.m_VB[meshA_.m_IB[i+1]];
				TriA[2] = meshA_.m_VB[meshA_.m_IB[i+2]];
				D3DXVec3TransformCoord(&TriB[0], &meshB_.m_VB[meshB_.m_IB[j]], &T);
				D3DXVec3TransformCoord(&TriB[1], &meshB_.m_VB[meshB_.m_IB[j+1]], &T);
				D3DXVec3TransformCoord(&TriB[2], &meshB_.m_VB[meshB_.m_IB[j+2]], &T);
				// �����Փ˂������Ȃ猵���Ƀ`�F�b�N����
				if(TriTriCoPosibility(
					TriA[0],
					TriA[1],
					TriA[2],
					TriB[0],
					TriB[1],
					TriB[2]
					) == false)
				{
					// ����Ɏg�����O�p�`���m���Փ˂̉\���������Ă���B
					// �Փˈʒu
					t_vector3 hitA;
					t_vector3 hitB;
					t_vector3 penetrate;
					if(TriTriCollidePos(
						TriA[0],
						TriA[1],
						TriA[2],
						TriB[0],
						TriB[1],
						TriB[2],
						&hitA,
						&hitB,
						penetrate_
						) == true)
					{				
						*colPosA_ = hitA;
						D3DXMatrixInverse(&tmp, NULL, &T);
						D3DXVec3TransformCoord(colPosB_,&hitA, &tmp);
				
						return true;
					}
 				}
			}
			return false;



			// �_�̈ʒu�Ƃ��̃X�C�[�v
			t_vector3 r;
			t_vector3 sweepVec;

	

			// A���猩��B�̒��S�ʒu�E���_�ʒu
			t_vector3 center = t_vector3(T._41, T._42, T._43);	// ���̂̒��S���Έʒu
	
			// �Փˈʒu
			t_vector3 colPos;

			for(int i = 0; i < meshA_.m_numIB - 3; i+=3)	// A�̃|���S���ɑ΂���
			for(int j = 0; j < meshB_.m_numIB; j++)	// B�̂��ׂĂ̒��_���h���邩�ǂ���
			{
				// planeN = t_vector3(1,0,0);
				// planeP = t_vector3(sizeA_.x*worldA_._11, sizeA_.y*worldA_._21, sizeA_.z*worldA_._31);
				// r = t_vector3(-sizeB_.x, -sizeB_.y, -sizeB_.z); 
				// r = pos + r;

				RotSweep(T, meshB_.m_VB[j], v, omega, &sweepVec);

				t_vector3 P;	//
				D3DXVec3TransformCoord(&P, &meshB_.m_VB[j], &T);// ���[�J���ł̊p�����x�Z�o
	
				t_vector3 Q = sweepVec+P;

				hit = TriSegCol(
					meshA_.m_VB[meshA_.m_IB[i]],
					meshA_.m_VB[meshA_.m_IB[i+1]],
					meshA_.m_VB[meshA_.m_IB[i+2]],
					P,
					Q,  // ���ʂ�����sweep�����łɁA���߂��Ă���̂ɓ����Ōv�Z�����
					&colPos);

		
				if(hit == true)
				{
					// *penetrate_ = sweepVec - colPos;	// ������v�Z���Ȃ���Ȃ�Ȃ��̂����ʂ�����
					// Cross(penetrate_,&meshA_.m_VB[meshA_.m_IB[i]],	&meshA_.m_VB[meshA_.m_IB[i+1]]);
					*penetrate_ = t_vector3(0,1,0);// *penetrate_ / D3DXVec3Length(penetrate_);
					*colPosA_ = P;

					D3DXVec3TransformCoord(colPosB_, &colPos, &worldA_);// meshB_.m_VB[j];
					return true;
				}
			}

			// �����܂ł����Ƃ������Ƃ͏Փ˂͕ӑΕӂ̂�


			// !!

			// for(i = 0; i < 6; i++)		// A�̃|���S���ɑ΂���
			// {
			// 	for(j = 0; j < 8; i++)	// B�̕�
			// 	{}
			// }






			return false;
		}
		/**********************************************************/
		/// **********************************************************/
		//// ���ꂼ��̃��[�J���ł̏Փˈʒu���o
		//// !!!�܂�A�������������m�ȏՓˈʒu����Ȃ��您
		// eSTATUS CClollisionFunc3D::MeshColPos(
		// 	const CTriMesh&		meshA_,
		// 	const CTriMesh&		meshB_,
		// 	const t_matrix4&	worldA_,				// ���ꂼ��̈ʒu�p��
		// 	const t_matrix4&	worldB_,
		// 	t_vector3*		colPosA_,		// A�̏Փˈʒu�iA���̒��S����̑��Έʒu�E��΍��W�j
		// 	t_vector3*		colPosB_,		// B�̏Փˈʒu�iB���̒��S����̑��Έʒu�E��΍��W�j
		// 	t_vector3*		penetrate_)		// �Փ˕����Ɛ[��
		// {
		// 	// ���΍��W�ɕϊ���B����A�̃��[�J����
		// 	t_matrix4 T;
		// 	t_matrix4 tmp;
		// 	D3DXMatrixInverse(&tmp, NULL, &worldA_);
		// 	D3DXMatrixMultiply(&T, &worldB_, &tmp);
		//
		// 	t_vector3 TriA[3];
		// 	t_vector3 TriB[3];
		// 	// A�̃|���S���ɑ΂���B�̃|���S���̌����\���`�F�b�N
		// 	// ���������Ēx���Ȃ邩��
		// 	for(int i = 0; i < meshA_.m_numIB - 3; i+=3)	
		// 	for(int j = 0; j < meshB_.m_numIB - 3; j+=3)		
		// 	{
		// 		// ���W�ϊ�
		// 		TriA[0] = meshA_.m_VB[meshA_.m_IB[i]];
		// 		TriA[1] = meshA_.m_VB[meshA_.m_IB[i+1]];
		// 		TriA[2] = meshA_.m_VB[meshA_.m_IB[i+2]];
		// 		D3DXVec3TransformCoord(&TriB[0], &meshB_.m_VB[meshB_.m_IB[j]], &T);
		// 		D3DXVec3TransformCoord(&TriB[1], &meshB_.m_VB[meshB_.m_IB[j+1]], &T);
		// 		D3DXVec3TransformCoord(&TriB[2], &meshB_.m_VB[meshB_.m_IB[j+2]], &T);
		// 		// �����Փ˂������Ȃ猵���Ƀ`�F�b�N����
		// 		if(TriTriCoPosibility(
		// 			TriA[0],
		// 			TriA[1],
		// 			TriA[2],
		// 			TriB[0],
		// 			TriB[1],
		// 			TriB[2]
		// 			) == false)
		// 		{
		// 			// ����Ɏg�����O�p�`���m���Փ˂̉\���������Ă���B
		// 			// �Փˈʒu
		// 			t_vector3 hitA;
		// 			t_vector3 hitB;
		// 			t_vector3 penetrate;
		// 			if(TriTriCollidePos(
		// 				TriA[0],
		// 				TriA[1],
		// 				TriA[2],
		// 				TriB[0],
		// 				TriB[1],
		// 				TriB[2],
		// 				&hitA,
		// 				&hitB,
		// 				penetrate_
		// 				) == true)
		// 			{				
		// 				*colPosA_ = hitA;
		// 				D3DXMatrixInverse(&tmp, NULL, &T);
		// 				D3DXVec3TransformCoord(colPosB_,&hitA, &tmp);
		// 				
		// 				return true;
		// 			}
		// 		}
		// 	}
		// 	return false;
		// }
		//// �ӂ̏Փ�
		//
		// 	if(hitCount1 > 0 && hitCount2 > 0)
		// 	{
		// 		// assert(hitCount2 == 1);		// !!!���_���Փ˂��Ă���Ƃ��͂�������肦�遨���x�Ώ�
		// 		switch(hitFlag2|hitFlag1)
		// 		{
		// 		case HIT_AB1 | HIT_AB2:
		// 			LineLineProximate(apexA1_, apexA2_ - apexA1_, apexB1_, apexB2_ - apexB1_, hitPos1_, hitPos2_);
		// 			break;
		// 		case HIT_AB1 | HIT_BC2:
		// 			LineLineProximate(apexA1_, apexA2_ - apexA1_, apexB2_, apexB3_ - apexB2_, hitPos1_, hitPos2_);
		// 			break;
		// 		case HIT_AB1 | HIT_CA2:
		// 			LineLineProximate(apexA1_, apexA2_ - apexA1_, apexB3_, apexB1_ - apexB3_, hitPos1_, hitPos2_);
		// 			break;
		//
		// 		case HIT_BC1 | HIT_AB2:
		// 			LineLineProximate(apexA3_, apexA2_ - apexA3_, apexB1_, apexB2_ - apexB1_, hitPos1_, hitPos2_);
		// 			break;
		// 		case HIT_BC1 | HIT_BC2:
		// 			LineLineProximate(apexA3_, apexA2_ - apexA3_, apexB2_, apexB3_ - apexB2_, hitPos1_, hitPos2_);
		// 			break;
		// 		case HIT_BC1 | HIT_CA2:
		// 			LineLineProximate(apexA3_, apexA2_ - apexA3_, apexB1_, apexB3_ - apexB1_, hitPos1_, hitPos2_);
		// 			break;
		//
		// 		case HIT_CA1 | HIT_AB2:
		// 			LineLineProximate(apexA1_, apexA3_ - apexA1_, apexB1_, apexB2_ - apexB1_, hitPos1_, hitPos2_);
		// 			break;
		// 		case HIT_CA1 | HIT_BC2:
		// 			LineLineProximate(apexA1_, apexA3_ - apexA1_, apexB3_, apexB2_ - apexB3_, hitPos1_, hitPos2_);
		// 			break;
		// 		case HIT_CA1 | HIT_CA2:
		// 			LineLineProximate(apexA1_, apexA3_ - apexA1_, apexB1_, apexB3_ - apexB1_, hitPos1_, hitPos2_);
		// 			break;
		// 		
		// 		default:
		// 			*hitPos1_ = ::t_vector3(0,0,0);
		// 			*hitPos2_ = ::t_vector3(0,0,0);
		// 			return false;
		// 			// RFASSERT(false, NULL);// �G���[�ӂ̏Փ˂��Ȃ��Ƃ�
		// 		}
		// 		
		// 		// �H�����ݎZ�o
		// 		*penetrate_ =  *hitPos2_ - *hitPos1_;
		// 		return HRK_COL_LINE;
		// 	}

					// if(TriLineCol(apexB1_, apexB2_, apexB3_, apexA3_, apexA3_+(depth * n), &gomi) == false)
					// 	return false;
		#endif

		///@brief	������-���̏Փ˂𔻒肷��
		///@param	result_    ���ʂ����镽��
		///@param	plane_     ���ɂȂ镽��
		///@param	transform_ �g�����X�t�H�[���s��
		void MakePlaneTransformed(
			CPlane*                 result_,
			const CPlane&			plane_,
			const math::t_matrix4&	transform_)
		{
			//�����Ԉ���Ă�B
			//���ʂ͉�]����Ƃ��@��̉�]�{��_�̉�]���K�v
			assert(false);
			MakeVectorCoordinateTransformed(&result_->normal(), transform_, plane_.normal());
		}


		///@brief	������-���̏Փ˂𔻒肷��
		///@param	frustum_	
		///@param	point_		
		///@param	radius_		���ʂ̖@�������I�t�Z�b�g
		///@retval	true  �{�b�N�X�����ʂ̗���
		///@retval	false �{�b�N�X�����ʂ̕\��
		bool FrustumSphereCol(
			const CFrustum&			frustum_,				
			const math::t_vector3&	point_,					
			float				    radius_)
		{
			t_vector3 pos;
			MakeVectorTransformed(&pos, frustum_.GetTransformInv(), point_);

			for (int i=CFrustum::NUM_PLANES-1; i>=0; i--)
			{
				// enum�L���X�g�I�I
				int warning;
				if (PlanePtDistance(frustum_.GetPlane((CFrustum::e_side_type)i), pos) > radius_)
					return false;
			}
			return true;
		}

		///@brief	������-���̏Փ˂𔻒肷��
		///@param	frustum_	
		///@param	point_		
		///@param	radius_		���ʂ̖@�������I�t�Z�b�g
		///@retval	<0	�{�b�N�X�����ʂ̗���
		///@retval	>0	�{�b�N�X�����ʂ̕\��
		///@retval	==0	���ʂƌ�������			
		void MakeFrustumTransformed(				
			CFrustum*               result_,	
			const CFrustum&			frustum_,				
			const math::t_matrix4&	transform_)
		{
			for (int i=CFrustum::NUM_PLANES; i>=0; i--)
			{
				// enum�L���X�g�I�I
				int warning;
				MakePlaneTransformed(
					&(result_->GetPlane((CFrustum::e_side_type)i)),
					frustum_.GetPlane((CFrustum::e_side_type)i),
					transform_);
			}
		}
	}
}
