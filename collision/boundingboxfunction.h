///
///@brief		プリミティブ衝突関連関数群
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
			///@brief	項等行列、零ベクトル等の定義
			// @{

			///@brief	無限遠を範囲にとる空のAABB
			static const CAabb2 g_aabb2_empty(
				math::t_vector2(
					-math::RF_BIG_NUM,
					-math::RF_BIG_NUM),
				math::t_vector2(
					math::RF_BIG_NUM,
					math::RF_BIG_NUM));
			// @}

		
			/////@brief	グローバル関数
			// namespace util
			// {
				///@defgroup	operation
				///@brief		幾何処理
				// @{
			
				///@brief	同次変換行列による変換を適用する
				///@note
				/// <br>
				/// 		x' = x*m11 + y*m21 + x*m31<br>
				/// <br>
				/// 	であり、<br>
				/// 	各行列要素がそれぞれ最小値になる→AABBの最小値<br>
				/// 	各行列要素がそれぞれ最大値になる→AABBの最大値<br>
				/// 	回転によっては元のAABBのminiが結果のAABBのminiに対応するとは限らないため、
				/// 	どちらが結果に対応するか判定する必要がある。<br>
				/// この関数の結果返ってくるAABBは元のAABB以上のサイズとなることに注意すること。<br>
				///
				///@param	m_		同次変換行列
				///@param	aabb_	AABB
				///@param	out_	[out]変換後のAABB
				inline void Transform(
					const CAabb2&			aabb_,
					const math::t_matrix3&	m_,
					CAabb2*					out_)
				{
					math::t_vector3 mini;

					// 平行移動成分をmini maxi の初期値とする
					math::GetTranslation(m_, &out_->mini());
					out_->maxi() = out_->mini();

					// 最小最大を畳み込む
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
		///@brief	項等行列、零ベクトル等の定義
		// @{

		///@brief	無限遠を範囲にとる空のAABB
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

		
		/////@brief	グローバル関数
		// namespace util
		// {
			///@defgroup	operation
			///@brief		幾何処理
			// @{
			
			///@brief	同次変換行列による変換を適用する
			///@note
			/// <br>
			/// 		x' = x*m11 + y*m21 + x*m31<br>
			/// <br>
			/// 	であり、<br>
			/// 	各行列要素がそれぞれ最小値になる→AABBの最小値<br>
			/// 	各行列要素がそれぞれ最大値になる→AABBの最大値<br>
			/// 	回転によっては元のAABBのminiが結果のAABBのminiに対応するとは限らないため、
			/// 	どちらが結果に対応するか判定する必要がある。<br>
			/// この関数の結果返ってくるAABBは元のAABB以上のサイズとなることに注意すること。<br>
			///
			///@param	m_		同次変換行列
			///@param	aabb_	AABB
			///@param	out_	[out]変換後のAABB
			inline void Transform(
				const CAabb3&			aabb_,
				const math::t_matrix4&	m_,
				CAabb3*					out_)
			{
				math::t_vector3 mini;
				// 平行移動成分をmini maxi の初期値とする
				math::MakeVectorPositionFromAffineMatrix(&out_->mini(), m_);
				out_->maxi() = out_->mini();

				// 最小最大を畳み込む
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