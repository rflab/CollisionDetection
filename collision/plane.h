/// rflab. All Rights Reserved.
///@brief	plane
///@auther	rflab.
///@date	2010/09/18
///@file	plane.h

#ifndef _RF_COLLISION_PLANE_H_
#define _RF_COLLISION_PLANE_H_

#include "../math/index.h"

// @namespace rf
namespace rf
{
	// @namespace collision
	namespace collision
	{
		///@brief nx*x + ny*y + nz*z + d =  0 
		/// 	���ʂ̖@���x�N�g����(a, b, c)
		class CPlane
		{
		private:
			/// (a*x + b*y+ c*z) + d*w = 0
			math::t_vector3 m_normal;
			float m_d;

		public:
			
			///@brief	�����Ȃ��R���X�g���N�^�͂Ȃɂ����Ȃ�
			CPlane()// :m_min(-RF_BIG_NUM),m_max(RF_BIG_NUM)
			{
			}
				
			///@brief	AABB�̒�`���s���R���X�g���N�^
			CPlane(float a_, float b_, float c_, float d_)
				:m_normal(a_, b_, c_), m_d(d_)
			{
			}
			
			///@brief	AABB�̒�`���s���R���X�g���N�^
			CPlane(
				const math::t_vector3& apex0,
				const math::t_vector3& apex1,
				const math::t_vector3& apex2)
			{
				Set(apex0, apex1, apex2);
			}

			void Set(
				const math::t_vector3& apex0,
				const math::t_vector3& apex1,
				const math::t_vector3& apex2)
			{
				math::t_vector3 ab = apex1 - apex0;
				math::t_vector3 ac = apex2 - apex0;
	
				// �@��
				math::MakeVectorCross(&m_normal, ab, ac);		
				m_normal.FastNormalize();

				// ���_����̋���
				m_d = Dot(m_normal, apex0);
			}
			
			// �A�N�Z�b�T
			const math::t_vector3& normal() const 
			{
				return m_normal;
			}
			
			math::t_vector3& normal()
			{
				return m_normal;
			}			

			float d() const
			{
				return m_d;
			}
			
			float& d()
			{
				return m_d;
			}


		};

	}	
}


#endif
