///
///@brief		�v���~�e�B�u�Փˊ֘A�֐��Q
///
///@auther		RockField
///@date		2010/04/29
///@file		boundingboxutil.h
///

#ifndef _RF_BOUNDINGBOX_util_H_
#define _RF_BOUNDINGBOX_util_H_

#include "../math/index.h"
#include "boundingbox.h"


///@brief rf
namespace rf
{
	namespace math
	{
		typedef collision::CAabb2 t_rect;
	}

	///@brief collision
	namespace collision
	{
		typedef CAabb2 t_aabb2;
		typedef CAabb3 t_aabb3; 
		#if 0
			///@name	definitions
			///@brief	�����s��A��x�N�g�����̒�`
			// @{

			///@brief	��������͈͂ɂƂ���AABB
			static const CAabb2 g_aabb2_empty(
				math::t_vector2(
					-math::RF_BIG_NUM,
					-math::RF_BIG_NUM),
				math::t_vector2(
					math::RF_BIG_NUM,
					math::RF_BIG_NUM));
			// @}

		
			/////@brief	�O���[�o���֐�
			// namespace util
			// {
				///@defgroup	operation
				///@brief		�􉽏���
				// @{
			
				///@brief	�����ϊ��s��ɂ��ϊ���K�p����
				///@note
				/// <br>
				/// 		x' = x*m11 + y*m21 + x*m31<br>
				/// <br>
				/// 	�ł���A<br>
				/// 	�e�s��v�f�����ꂼ��ŏ��l�ɂȂ遨AABB�̍ŏ��l<br>
				/// 	�e�s��v�f�����ꂼ��ő�l�ɂȂ遨AABB�̍ő�l<br>
				/// 	��]�ɂ���Ă͌���AABB��mini�����ʂ�AABB��mini�ɑΉ�����Ƃ͌���Ȃ����߁A
				/// 	�ǂ��炪���ʂɑΉ����邩���肷��K�v������B<br>
				/// ���̊֐��̌��ʕԂ��Ă���AABB�͌���AABB�ȏ�̃T�C�Y�ƂȂ邱�Ƃɒ��ӂ��邱�ƁB<br>
				///
				///@param	m_		�����ϊ��s��
				///@param	aabb_	AABB
				///@param	out_	[out]�ϊ����AABB
				inline void Transform(
					const CAabb2&			aabb_,
					const math::t_matrix3&	m_,
					CAabb2*					out_)
				{
					math::t_vector3 mini;

					// ���s�ړ�������mini maxi �̏����l�Ƃ���
					math::GetTranslation(m_, &out_->mini());
					out_->maxi() = out_->mini();

					// �ŏ��ő����ݍ���
					for (int c=0; c<2; c++)
					for (int r=0; r<2; r++)
					{
						if (m_(r, c) > 0.0f)
						{
							out_->mini(c) += aabb_.mini(r) * m_(r, c);
							out_->maxi(c) += aabb_.maxi(r) * m_(r, c);
						}									
						else								
						{									
							out_->mini(c) += aabb_.maxi(r) * m_(r, c);
							out_->maxi(c) += aabb_.mini(r) * m_(r, c);
						}
					}
				}

				// @}
			// }
		}
	#endif
	
		///@name	definitions
		///@brief	�����s��A��x�N�g�����̒�`
		// @{

		///@brief	��������͈͂ɂƂ���AABB
		static const CAabb3 g_aabb3_empty(
			math::t_vector3(
				-math::RF_BIG_NUM,
				-math::RF_BIG_NUM,
				-math::RF_BIG_NUM),
			math::t_vector3(
				math::RF_BIG_NUM,
				math::RF_BIG_NUM,
				math::RF_BIG_NUM));
		// @}

		
		/////@brief	�O���[�o���֐�
		// namespace util
		// {
			///@defgroup	operation
			///@brief		�􉽏���
			// @{
			
			///@brief	�����ϊ��s��ɂ��ϊ���K�p����
			///@note
			/// <br>
			/// 		x' = x*m11 + y*m21 + x*m31<br>
			/// <br>
			/// 	�ł���A<br>
			/// 	�e�s��v�f�����ꂼ��ŏ��l�ɂȂ遨AABB�̍ŏ��l<br>
			/// 	�e�s��v�f�����ꂼ��ő�l�ɂȂ遨AABB�̍ő�l<br>
			/// 	��]�ɂ���Ă͌���AABB��mini�����ʂ�AABB��mini�ɑΉ�����Ƃ͌���Ȃ����߁A
			/// 	�ǂ��炪���ʂɑΉ����邩���肷��K�v������B<br>
			/// ���̊֐��̌��ʕԂ��Ă���AABB�͌���AABB�ȏ�̃T�C�Y�ƂȂ邱�Ƃɒ��ӂ��邱�ƁB<br>
			///
			///@param	m_		�����ϊ��s��
			///@param	aabb_	AABB
			///@param	out_	[out]�ϊ����AABB
			inline void Transform(
				const CAabb3&			aabb_,
				const math::t_matrix4&	m_,
				CAabb3*					out_)
			{
				math::t_vector3 mini;
				// ���s�ړ�������mini maxi �̏����l�Ƃ���
				math::MakeVectorPositionFromAffineMatrix(&out_->mini(), m_);
				out_->maxi() = out_->mini();

				// �ŏ��ő����ݍ���
				for (int c=0; c<3; c++)
				for (int r=0; r<3; r++)
				{
					if (m_(r, c) > 0.0f)
					{
						out_->mini(c) += aabb_.mini(r) * m_(r, c);
						out_->maxi(c) += aabb_.maxi(r) * m_(r, c);
					}									
					else								
					{									
						out_->mini(c) += aabb_.maxi(r) * m_(r, c);
						out_->maxi(c) += aabb_.mini(r) * m_(r, c);
					}
				}
			}

			// @}

		// }
	}
}
#endif