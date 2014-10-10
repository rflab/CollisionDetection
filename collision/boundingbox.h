///
///@brief		バウンディングボックス
/// 					- Axis Aligned Bounding Box(Aabb)
/// 					- Oriented Bounding Box(Obb)
/// 				mini-maxi型のAABBを基本とする
///
///@auther		RockField
///@date		2010/04/30
///@file		boundingbox.h
///

#ifndef _RF_BOUNDONG_BOX_H_
#define _RF_BOUNDONG_BOX_H_

#include "../math/index.h"

///@brief rf
namespace rf
{
	///@brief collision
	namespace collision
	{
		///@brief	二次元軸並行バウンディングボックス
		class CAabb2
		{
		public:

			typedef math::t_vector2::value_type value_type;
		
		private:

			math::t_vector2 m_min;
			math::t_vector2 m_max;
		
		public:
		
			///@name	constructor
			// @{
				///@brief	引数なしコンストラクタはなにもしない
				CAabb2()// :m_min(-RF_BIG_NUM),m_max(RF_BIG_NUM)
				{
				}
				
				///@brief	AABBの定義も行うコンストラクタ
				CAabb2(const math::t_vector2& min_, const math::t_vector2& max_)
					:m_min(min_),m_max(max_)
				{
				}

				///@brief	AABBの定義も行うコンストラクタ
				CAabb2(float l_, float t_, float r_, float b_)
					:m_min(l_, t_),m_max(r_, b_)
				{
				}
			// @}

			///@name	property
			// @{
				///@brief	AABBのサイズを取得する	
				math::t_vector2 GetSize() const
				{
					return m_max - m_min;
				}

				///@brief	AABBのx軸方向サイズを取得する
				value_type sx() const
				{
					return m_max(0) - m_min(0);
				}

				///@brief	AABBのy軸方向サイズを取得する
				value_type sy() const
				{
					return m_max(1) - m_min(1);
				}

				///@brief	AABBのx軸方向サイズを取得する
				value_type left() const
				{
					return m_min(0);
				}

				///@brief	AABBのy軸方向サイズを取得する
				value_type top() const
				{
					return m_min(1);
				}
				
				///@brief	AABBのx軸方向サイズを取得する
				value_type right() const
				{
					return m_max(0);
				}

				///@brief	AABBのy軸方向サイズを取得する
				value_type bottom() const
				{
					return m_max(1);
				}

				///@brief	AABBの重心を得る
				math::t_vector2 GetCenter() const
				{
					return (m_max + m_min) * value_type(0.5);
				}

				///@brief	最小点への参照を取得する
				value_type& mini(unsigned int ix_)
				{
					assert(ix_ < 2);
					return m_min(ix_);
				}

				///@brief	最大点への参照を取得する
				value_type& maxi(unsigned int ix_)
				{
					assert(ix_ < 2);
					return m_max(ix_);
				}
				
				///@brief	最小点への参照を取得する
				const value_type& mini(unsigned int ix_) const 
				{
					assert(ix_ < 2);
					return m_min(ix_);
				}

				///@brief	最大点への参照を取得する
				const value_type& maxi(unsigned int ix_) const
				{
					assert(ix_ < 2);
					return m_max(ix_);
				}
				
				///@brief	最小点への参照を取得する
				math::t_vector2& mini()
				{
					return m_min;
				}

				///@brief	最大点への参照を取得する
				math::t_vector2& maxi()
				{
					return m_max;
				}

				///@brief	最小点への参照を取得する
				const math::t_vector2& mini() const 
				{
					return m_min;
				}

				///@brief	最大点への参照を取得する
				const math::t_vector2& maxi() const
				{
					return m_max;
				}
				
				///@brief	ix_で指定されたcornerの座標を返す
				///
				/// 				2 ______3
				/// 				 |     |
				/// 	   y		 |     | 
				/// 	   |		 |_____|	
				/// 	   |___x	0		1	
				/// 
 				math::t_vector2 operator()(unsigned int ix_)
 				{
					assert(ix_ < 4);

					return math::t_vector2(
						(ix_&(1 << 0)) ? m_max(0) : m_min(0),
						(ix_&(1 << 1)) ? m_max(1) : m_min(1));
				}
			// @}

				
			///@name	transformation
			// @{
				///@brief	頂点を追加し、必要があればAABBのサイズを変更する
				void AddPt(const math::t_vector2& p_)
				{
					if (p_(0) < m_min(0)) m_min(0) = p_(0);
					if (p_(0) > m_max(0)) m_max(0) = p_(0);
					if (p_(1) < m_min(1)) m_min(1) = p_(1);
					if (p_(1) > m_max(1)) m_max(1) = p_(1);
				}

				///@brief	AABBを追加し、必要があればAABBのサイズを変更する
				void MargeAABB(const CAabb2& aabb_)
				{
					if (aabb_.mini(0) < m_min(0)) m_min(0) = aabb_.mini(0);
					if (aabb_.maxi(0) > m_max(0)) m_max(0) = aabb_.maxi(0);
					if (aabb_.mini(1) < m_min(1)) m_min(1) = aabb_.mini(1);
					if (aabb_.maxi(1) > m_max(1)) m_max(1) = aabb_.maxi(1);
				}
			// @}
				
		};

		///@brief		二次元有向バウンディングボックス
		///@attention	作りかけ
		class CObb2
		{

		};

		
		///@brief	三次元軸並行バウンディングボックス
		class CBoundingSphere3
		{
		private:

			/// 位置
			math::t_vector3 m_position;

			/// 半径
			float m_radius;

		public:

			CBoundingSphere3(const math::t_vector3& pos_, float radius_)
				:
				m_position(pos_),
				m_radius(radius_)
			{
			}
				
			//--------------------------- propaties
			float& r()
			{
				return m_radius;
			}
			
			float& x()
			{
				return m_position(0);
			}

			float& y()
			{
				return m_position(1);
			}

			float& z()
			{
				return m_position(2);
			}
			
			math::t_vector3& pos()
			{
				return m_position;
			}


			//--------------------------- const access
			float r() const
			{
				return m_radius;
			}
			
			float x() const
			{
				return m_position(0);
			}

			float y() const
			{
				return m_position(1);
			}

			float z() const
			{
				return m_position(2);
			}
			
			const math::t_vector3& pos() const
			{
				return m_position;
			}
		};
		///@brief	三次元軸並行バウンディングボックス
		class CAabb3
		{
		public:
			typedef math::t_vector3::value_type value_type;
		
		private:
			math::t_vector3 m_min;
			math::t_vector3 m_max;

		public:
			///@name	constructor
			// @{
				///@brief	引数なしコンストラクタはなにもしない
				CAabb3()// :m_min(-RF_BIG_NUM),m_max(RF_BIG_NUM)
				{
				}
				
				///@brief	AABBの定義も行うコンストラクタ
				CAabb3(const math::t_vector3& min_, const math::t_vector3& max_)
					:m_min(min_),m_max(max_)
				{
				}
			// @}

			///@name	property
			// @{
				///@brief	AABBのサイズを取得する	
				math::t_vector3 GetSize() const
				{
					return m_max - m_min;
				}

				///@brief	AABBのx軸方向サイズを取得する
				value_type sx() const
				{
					return m_max(0) - m_min(0);
				}

				///@brief	AABBのy軸方向サイズを取得する
				value_type sy() const
				{
					return m_max(1) - m_min(1);
				}

				///@brief	AABBのy軸方向サイズを取得する
				value_type sz() const
				{
					return m_max(2) - m_min(2);
				}

				///@brief	AABBの重心を得る
				math::t_vector3 GetCenter() const
				{
					return (m_max + m_min) * value_type(0.5);
				}

				///@brief	最小点への参照を取得する
				value_type& mini(unsigned int ix_)
				{
					assert(ix_ < 3);
					return m_min(ix_);
				}

				///@brief	最大点への参照を取得する
				value_type& maxi(unsigned int ix_)
				{
					assert(ix_ < 3);
					return m_max(ix_);
				}

				///@brief	最小点への参照を取得する
				const value_type& mini(unsigned int ix_) const
				{
					assert(ix_ < 3);
					return m_min(ix_);
				}

				///@brief	最大点への参照を取得する
				const value_type& maxi(unsigned int ix_) const
				{
					assert(ix_ < 3);
					return m_max(ix_);
				}
								
				///@brief	最小点への参照を取得する
				math::t_vector3& mini()
				{
					return m_min;
				}

				///@brief	最大点への参照を取得する
				math::t_vector3& maxi()
				{
					return m_max;
				}

				///@brief	最小点への参照を取得する
				const math::t_vector3& mini() const 
				{
					return m_min;
				}

				///@brief	最大点への参照を取得する
				const math::t_vector3& maxi() const
				{
					return m_max;
				}
				///@brief	ix_で指定されたcornerの座標を返す
				///
				/// 	              6______7	
				/// 				  /|    /|
				/// 				2/_____/3|
				/// 				 | |___|_|
				/// 	   y		 |4/   | /5 
				/// 	   | z		 |/____|/	
				/// 	   |/__x	0		 1	
				/// 
 				math::t_vector3 operator()(unsigned int ix_)
 				{
					assert(ix_ < 8);

					return math::t_vector3(
						(ix_&(1 << 0)) ? m_max(0) : m_min(0),
						(ix_&(1 << 1)) ? m_max(1) : m_min(1),
						(ix_&(1 << 2)) ? m_max(2) : m_min(2));
				}
			// @}

				
			///@name	transformation
			// @{
				///@brief	頂点を追加し、必要があればAABBのサイズを変更する
				void AddPt(const math::t_vector3& p_)
				{
					if (p_(0) < m_min(0)) m_min(0) = p_(0);
					if (p_(0) > m_max(0)) m_max(0) = p_(0);
					if (p_(1) < m_min(1)) m_min(1) = p_(1);
					if (p_(1) > m_max(1)) m_max(1) = p_(1);
					if (p_(2) < m_min(2)) m_min(2) = p_(2);
					if (p_(2) > m_max(2)) m_max(2) = p_(2);
				}

				///@brief	AABBを追加し、必要があればAABBのサイズを変更する
				void MargeAABB(const CAabb3& aabb_)
				{
					if (aabb_.mini(0) < m_min(0)) m_min(0) = aabb_.mini(0);
					if (aabb_.maxi(0) > m_max(0)) m_max(0) = aabb_.maxi(0);
					if (aabb_.mini(1) < m_min(1)) m_min(1) = aabb_.mini(1);
					if (aabb_.maxi(1) > m_max(1)) m_max(1) = aabb_.maxi(1);
					if (aabb_.mini(2) < m_min(2)) m_min(2) = aabb_.mini(2);
					if (aabb_.maxi(2) > m_max(2)) m_max(2) = aabb_.maxi(2);
				}
			// @}
		};

		///@brief		三次元有向バウンディングボックス
		///@attention	作りかけ
		class CObb3
		{
		private:
			CAabb3			m_aabb;
			math::t_matrix4	m_world;
		};
	}
}
#endif
