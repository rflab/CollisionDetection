/// rflab. All Rights Reserved.
///@brief	plane
///@auther	rflab.
///@date	2010/09/18
///@file	plane.h

#ifndef _RF_COLLISION_RAY_H_
#define _RF_COLLISION_RAY_H_

#include "../math/index.h"

// @namespace rf
namespace rf
{
	// @namespace collision
	namespace collision
	{
		///@brief start + dir
		class CRay
		{
		private:

			/// 始点
			math::t_vector3 m_start;
			/// 終点
			math::t_vector3 m_end;

		public:
			
			///@brief	AABBの定義も行うコンストラクタ
			CRay(const math::t_vector3 start_, const math::t_vector3 end_)
				:
				m_start(start_),
				m_end(end_)
			{
			}
			
			///
			const math::t_vector3& start() const {return m_start;}
			///
			math::t_vector3& start(){return m_start;}
			///
			const math::t_vector3& end() const {return m_end;}	
			///
			math::t_vector3& end(){return m_end;}
		};

	}	
}


#endif
