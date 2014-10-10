/// rflab. All Rights Reserved.
///@brief	frustum
///@auther	rflab.
///@date	2010/09/18
///@file	frustum.h

#ifndef _RF_COLLISION_FRUSTUM_H_
#define _RF_COLLISION_FRUSTUM_H_

#include "../config.h"
#include "../math/index.h"
#include "plane.h"

// @namespace rf
namespace rf
{
	// @namespace collision
	namespace collision
	{
		///@brief	������
		/// 			�R���{���͓����ϊ���ɂ��΂�����łȂ��H
		///@attention ����ō����������̓��[���h�g�����X�t�H�[������Ă��Ȃ�
		///            �K�v�������Set�֐����C�����ׂ�
		class CFrustum
		{
		public:
			typedef enum
			{
				PLANE_NEAR = 0,
				PLANE_FAR,
				PLANE_LEFT,
				PLANE_TOP,
				PLANE_RIGHT,
				PLANE_BOTTOM,
				NUM_PLANES 
			}e_side_type;
			
		private:
			
			math::t_matrix4 m_transform;
			math::t_matrix4 m_transformInv;

			CPlane m_sides[NUM_PLANES];
			math::t_vector3 m_nearClipvertices[4];
			math::t_vector3 m_farClipVertices[4];
			float m_fov;
			float m_aspect;
			float m_near;
			float m_far;

		public:
			
			/// �������Ȃ��R���X�g���N�^
			CFrustum()
			{
			}
			
			///@brief
			CFrustum(
				float fov_,    // = math::RF_PI/4.0f, // ����p90�x
				float aspect_, // = 4.0f/3.0f,
				float near_,   // = 1.0f,
				float far_,	   // = 1000.0f)
				const math::t_matrix4& transform_)
			{
				Set(m_fov, m_aspect, m_near, m_far);
				SetTransform(transform_);
			}
		
			//-----------------------------------------------property
			float GetFov() const
			{
				return m_fov;
			}

			float GetAspect() const
			{
				return m_aspect;
			}
		
			float GetNear() const
			{
				return m_near;
			}
			
			float GetFar() const
			{
				return m_far;
			}
			
			const math::t_matrix4& GetTransform() const
			{
				return m_transform;
			}
	
			const math::t_matrix4& GetTransformInv() const
			{
				return m_transformInv;
			}
			
			///@brief �\�����ʂ𓾂�
			const CPlane& GetPlane(e_side_type side_) const
			{
				return m_sides[side_];
			}
			
			///@brief �\�����ʂ𓾂�
			CPlane& GetPlane(e_side_type side_)
			{
				return m_sides[side_];
			}

			void SetTransform(const math::t_matrix4& transform_)
			{
				// ��{�I�ɉۃ����O�Ɏg���̂ŋt�s���ێ����Ă���
				m_transform = transform_;
				math::MakeMatrixInverseFastAffine(&m_transformInv, transform_);
			}
			
			//-----------------------------------------------
			// ��������쐬
			void Set(float fov_, float aspect_, float near_, float far_)
			{
				const math::t_vector3 RIGHT( 1.0f, 0, 0);    // x
				const math::t_vector3 UP(0, 1.0f, 0);        // y
				const math::t_vector3 FORWARD(0, 0, 1.0f);   // z
				const math::t_vector3 ORIGIN(0, 0, 0.0f);    // 0
				m_fov = fov_;
				m_aspect = aspect_;// height_/width_;
				m_near = near_;
				m_far = far_;

				float tanFov = math::Tan(fov_/2.0f);
				math::t_vector3 nearUp    = (near_ * tanFov) * UP;
				math::t_vector3 nearRight = (near_ * tanFov) * aspect_ * RIGHT;
				math::t_vector3 farUp	  = (far_ * tanFov) * UP;
				math::t_vector3 farRight  = (far_ * tanFov) * aspect_ * RIGHT;

				
				///@attention ����ō����������̓��[���h�g�����X�t�H�[������Ă��Ȃ�
				///            �K�v�������Set�֐����C�����ׂ�
				///�����Ńx�N�^����]����΂���

				// Z��O�ʂƒ�`����->�O���[�o���Ɏ����Ă����ق��������̂��H

				m_nearClipvertices[0] = (near_ * FORWARD) - nearRight + nearUp;
				m_nearClipvertices[1] = (near_ * FORWARD) + nearRight + nearUp;
				m_nearClipvertices[2] = (near_ * FORWARD) + nearRight - nearUp;
				m_nearClipvertices[3] = (near_ * FORWARD) - nearRight - nearUp;
				m_farClipVertices[0] = (far_ * FORWARD) - farRight + farUp;
				m_farClipVertices[1] = (far_ * FORWARD) + farRight + farUp;
				m_farClipVertices[2] = (far_ * FORWARD) + farRight - farUp;
				m_farClipVertices[3] = (far_ * FORWARD) - farRight - farUp;
				
				// ���ʂ��쐬
				m_sides[PLANE_NEAR  ].Set(m_nearClipvertices[0], m_nearClipvertices[1], m_nearClipvertices[2]);
				m_sides[PLANE_FAR   ].Set(m_farClipVertices[2], m_farClipVertices[1], m_farClipVertices[0]);
				m_sides[PLANE_LEFT  ].Set(m_farClipVertices[3], m_farClipVertices[0], ORIGIN);
				m_sides[PLANE_TOP   ].Set(m_farClipVertices[0], m_farClipVertices[1], ORIGIN);
				m_sides[PLANE_RIGHT ].Set(m_farClipVertices[1], m_farClipVertices[2], ORIGIN);
				m_sides[PLANE_BOTTOM].Set(m_farClipVertices[2], m_farClipVertices[3], ORIGIN);
			}

			#if 0
				CPlane& Inside(const t_vector3& pos_, float radius_)
				{	
					math::t_vector3 pos;
					math::MakeVectorTransformed(&pos, m_transformInv, pos_);
					FrustumSphereCol(m_camera->GetViewFrustum(), pos, radius_);
					return m_sides[side_];
				}
			#endif
			
			///@brief ���_�𓾂�
			/// ���_�ԍ��͌��J�������Ȃ��Ȃ�
			const math::t_vector3& GetVertex(unsigned int ix_) const
			{
				int warning_near_far;
				if (ix_ >= 8)
				{
					assert(false && "frustum verticies index over\n");
					return m_farClipVertices[0]; //���傤���Ȃ��̂łƂ肠����0�̂�Ԃ�
				}

				if (ix_ >= 4)
				{
					return m_farClipVertices[ix_-4];
				}
				else
				{
					return m_nearClipvertices[ix_];
				}
			}
		};
	}	
}


#endif
