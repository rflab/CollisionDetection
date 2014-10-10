// 衝突判定クラス
// rf. All Right Reserves.

// 2008/06/03　OBB追加＆全部作り直し

#ifndef _RF_COLLISION_TRIANGLE_H_
#define _RF_COLLISION_TRIANGLE_H_

#include <rflib/math/index.h>

///@brief rf
namespace rf
{
	///@brief collision
	namespace collision
	{
		///本当にただの三角形。インデックス等は使っていない
		template<class Vert>
		class TTriangle
		{
		public:
			///@brief 頂点
			Vert vertex[3];
		};

		///三角形頂点インデックス
		template<class Vert>
		class CTriangleIndex
		{
		public:
			///@brief 頂点配列
			///boost::optional< sp<vector<Vert> > vertices;

			///@brief 頂点インデックス
			int index[3];
		};
		
		#if 0
			// 最終形態
			// あらゆる凸メッシュを判定できるようにする予定
			class CColMesh3D : public CCollision3D
			{
			private:

			public:
				float			m_radius;			// 半径			
				CTriMesh	m_IndexedPrimitive;	// 頂点配列	
				D3DXMATRIX		m_world;			// 同時変換行列：ボックスの位置は中心点を基準とする
				D3DXVECTOR3		m_v;				// 速度
				D3DXVECTOR3		m_omega;			// 角速度

				CColMesh3D(const CTriMesh& mesh_, float BSphereR = 0);
				virtual ~CColMesh3D();

				virtual void SetWorld(D3DXMATRIX &world_){m_world = world_;}
				virtual const D3DXMATRIX& GetWorld(){return m_world;}

				void Draw();

				// 以下はタイプ違いのオーバーロード沢山
				virtual e_rf_flag MeshCol(
					CColMesh3D& obj_,
					D3DXVECTOR3* colPosA_,
					D3DXVECTOR3* colPosB_,
					D3DXVECTOR3* dir_);

			};
		#endif
	}
}
#endif
