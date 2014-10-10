// 線形8分木空間分割 Octree
// 作成者　	Iwahara Hiroaki
// 071026	作成

#ifndef _RF_COLLISION_OCTREE_H_
#define _RF_COLLISION_OCTREE_H_

#include <vector>
#include <stack>
#include <deque>
#include <list>
#include <set>
#include <utility>
#include <algorithm>
#include <exception>
#include <cassert>


// for memcpy
#include <string.h>
#include <rflib/common/index.h>

#include "boundingbox.h"

///@brief rf
namespace rf
{
	///@brief collision
	namespace collision
	{		
		// オブジェクトコンテナ用リストノード
		template<class T>
		class TOctreeNode
		{
			typedef std::list<TOctreeNode<T> > cell_type;
			typedef TOctreeNode<T> self_type;
			


		protected:

			///@brief このセルを登録した空間
			cell_type* m_cell;
						
			///@brief 判定対象オブジェクト
			sp<T> m_obj;
			
		public:

			///@brief コンストラクタ
			TOctreeNode()
				:
				m_cell(NULL),
				m_obj(NULL),
				m_type(INVALID)
			{
			}

			///@brief コンストラクタ
			explicit TOctreeNode(sp<T> obj_)
				:
				m_cell(NULL),
				m_obj(obj_),
				m_type(INVALID)
			{
			}
		
			///@brief コンストラクタ
			TOctreeNode(const self_type& obj_)
			{
				m_cell = obj_.m_cell;
				m_obj  = obj_.m_obj;
				m_type = obj_.m_type;
			}

			///@brief デストラクタ
			virtual ~TOctreeNode()
			{
				Remove();
			}

			///@brief 代入
			self_type& operator = (const self_type& obj_)
			{
				m_cell = obj_.m_cell;
				m_obj  = obj_.m_obj;
				m_type = obj_.m_type;
				return *this;
			}
			
			///@brief セルに登録している？
			bool IsValid()
			{
				return NULL != m_cell;
			}
			
			///@brief セルに登録している？
			void SetCell(cell_type* cell_)
			{
				m_cell = cell_;
			}
			
			///@brief 登録先のセルを取得する
			cell_type* GetCell()
			{
				return m_cell;
			}
			
			///@brief オブジェクト登録
			void SetObj(sp<T> obj_)
			{
				m_obj = obj_;
			}

			///@brief 登録されているオブジェクトを返す
			sp<T> GetObj()
			{
				return m_obj;
			}

			///@brief ??
			void Remove()
			{
				int warning;
				m_cell->Remove(sp<self_type>(this));
				m_cell = NULL;
			}
		};
		

		///@brief 衝突空間
		template <class T>
		class TOctreeSpace
		{
			
			///@brief 登録するセルのタイプ
			typedef TOctreeNode<T> node_type;

			///@brief 登録するセルのタイプ
			typedef std::list<sp<node_type> > cell_type;
											
			///@brief 衝突タイプ
			typedef enum			
			{
				POINT,
				CUBE,
				INVALID
			}collision_type;
			
			///@brief 衝突タイプ
			collision_type m_type;

			///@brief 衝突タイプを設定する
			void SetCollisionType(collision_type type_)
			{
				m_type = type_;
			}

			///@brief 衝突タイプを取得する
			collision_type GetCollisionType()
			{
				return m_type;
			}

		private:
			
			///@brief コンストラクタ抑止
			TOctreeSpace(){}
			
			///@brief コピーコンストラクタ抑止
			TOctreeSpace(const TOctreeSpace& obj_){}

		protected:
			
			///@brief 空間分割の深さ（分割レベル）
			unsigned int m_divisionLevel;			
			
			///@brief 領域の幅サイズ
			math::t_vector3 m_spaceSize;

			///@brief 領域の端の位置
			math::t_vector3 m_spaceOrigin;

			///@brief 最小セルの幅
			math::t_vector3 m_cellSize;			
				 
			///@brief 空間の数				 
			unsigned int m_numCell;

			///@brief 空間セルへのポインタ配列
			cell_type* m_cellArray;

			///@brief 衝突階層探索用デック
			std::deque<T*> m_tmpColStack;
			
		protected:
			
			///@brief モートン番号からセル配列インデックスを高速に取得する
			///@note 
			// (x_,y_,z_)番目の点のモートン番号
			// 	000? 00?0 00?0 0?00 ?00? 00?0 0?00 ?00?
			// 	00?0 0?00 ?00? 00?0 0?00 0?00 ?00? 00?0
			// 	0?00 ?00? 00?0 0?00 ?000 ?00? 00?0 0?00
			unsigned int Space2Morton(
				unsigned int x_,
				unsigned int y_,
				unsigned int z_)
			{
				unsigned int x = x_;
				x = (x|(x<<8)) & 0x0000f00f; // 0000 xxxx 0000 0000 1111 0000 0000 1111 ↓
				x = (x|(x<<4)) & 0x000c30c3; // xx00 00xx 0000 1100 0011 0000 1100 0011 ↓
				x = (x|(x<<2)) & 0x00249249; // 0x00 x00x 0010 0100 1001 0010 0100 1001

				unsigned int y = y_;
				y = (y|(y<<8)) & 0x0000f00f;
				y = (y|(y<<4)) & 0x000c30c3;
				y = (y|(y<<2)) & 0x00249249;
				
				unsigned int z = z_;
				z = (z|(y<<8)) & 0x0000f00f;
				z = (z|(y<<4)) & 0x000c30c3;
				z = (z|(y<<2)) & 0x00249249;

				return x | (y<<1) | (z<<2);
			}

			///@brief セル配列インデックスからモートン番号を高速に取得する
			// 0000 xxxx 0000 0000 1111 0000 0000 1111
			// xx00 0011 0000 1100 0011 0000 1100 0011
			// 0x00 1001 0010 0100 1001 0010 0100 1001
			void Morton2Space(
				unsigned int moton_,
				unsigned int* x_,
				unsigned int* y_,
				unsigned int* z_)
			{
				unsigned int x = moton_ & 0x00249249;
				x   = ((x|(x>>2)) & 0x000c30c3);
				x   = ((x|(x>>4)) & 0x0000f00f);
				*x_ = ((x|(x>>8)) & 0x000000ff);
		
				unsigned int y = ((moton_>>1) & 0x00249249);
				y   = ((y|(y>>2)) & 0x000c30c3);
				y   = ((y|(y>>4)) & 0x0000f00f);
				*y_ = ((y|(y>>8)) & 0x000000ff);
				
				unsigned int z = ((moton_>>2) & 0x00249249);
				z   = ((z|(y>>2)) & 0x000c30c3);
				z   = ((z|(y>>4)) & 0x0000f00f);
				*z_ = ((z|(y>>8)) & 0x000000ff);
			}

			///@brief バウンディングボックスの所属する空間番号
			bool GetSpaceIndex(
				const CAabb3& aabb_,
				unsigned int* ix_,
				unsigned int* moton_,
				unsigned int* level_)
			{
				// バウンディングボックスの最小・最大
				int xl = (int)((aabb_.mini(0) - m_regionOrigin(0)) / m_cellSize(0));
				int yl = (int)((aabb_.mini(0) - m_regionOrigin(1)) / m_cellSize(1));
				int zl = (int)((aabb_.mini(0) - m_regionOrigin(2)) / m_cellSize(2));
				int xm = (int)((aabb_.maxi(0) - m_regionOrigin(0)) / m_cellSize(0));
				int ym = (int)((aabb_.maxi(0) - m_regionOrigin(1)) / m_cellSize(1));
				int zm = (int)((aabb_.maxi(0) - m_regionOrigin(2)) / m_cellSize(2));

				// モートンナンバーの取得
				unsigned int minMoton = Space2Morton(xl, yl, zl);
				unsigned int maxMoton = Space2Morton(xm, ym, zm);
						
				// 親空間番号だけ抜き出す
				// 親空間部分が共通な場合、その部分は排他的論理和で0になる
				// たとえば排他的論理輪と0xffffff00との論理積が0ならもっと高レベル
				// TODO ビット演算とかで高速に一発で計算する方法がありそうな気がするが思いつかなかった。（割り算必須かしら）
				unsigned int xorMoton = l ^ m;
				assert(xorMoton & ~((0x1 << (m_divisionLevel*3))-1) != 0 && "範囲外");
				unsigned int level = m_divisionLevel;
				for (int i=0; i > m_divisionLevel; i++)
				{
					if (xorMoton & ~((0x1 << (i*3))-1) != 0 )
					{
						break;
					}
					level--;
				}
				unsigned int parentMoton = maxMoton >> ((m_divisionLevel - level)*3);
								
				// モートンナンバーから配列のインデックス、
				// それより上の階層に存在しえるセルの数を足すためのテーブル
				// 公比9の等比数列の和(8^n-1)/(8-1)
				const int OFFSETS[] = {0, 1, 1+8, 1+8+64, 1+8+64+512, 1+8+64+512+4096, 1+8+64+512+32768, 1+8+64+512++32768+262144};
				unsigned int ix = parentMoton + OFFSETS[level];		

				*level_ = level;
				*moton_ = motonNum;
				*ix_ = ix;
				return true;
			}

			///@brief 衝突判定を実行して、衝突ペアを取得する再帰関数
			bool GetFullCollisionVect_Recursive(unsigned int ix_, std::set<std::pair<sp<T>, sp<T> > >* retVect_)
			{
				// 衝突判定ペアリストイテレータ
				std::deque<T*>::iterator it;

				// コリジョンセル巡回用イテレータ
				cell_type::iterator cellitA = (m_cellArray[ix_])->begin();

				// 同一空間
				for (;cellitA != m_cellArray[ix_]->end(); cellitA++)
				{
					cell_type::iterator cellitB = cellitA;
					cellitB++;
					
					// 同じ空間の分を衝突組み合わせ配列に追加
					for (;cellitB != m_cellArray[ix_]->end();cellitB++)
					{
						//
						rfinfo("登録時衝突判定したい");

						retVect_->push_back(cellitA->GetObj());
						retVect_->push_back(cellitB->GetObj());
					}

					// スタックの分を衝突組み合わせ配列に追加
					for(it=m_tmpColStack.begin(); it!=m_tmpColStack.end(); it++)
					{
						retVect_->push_back(cellitA->GetObj());
						retVect_->push_back(*it);
					}
				}

				
				// 持ってるものをスタックに詰め込む
				unsigned int numObj = 0;
				for (cellitA = (m_cellArray[ix_])->begin(); cellitA != m_cellArray[ix_]->end(); cellitA++)
				{
					m_tmpColStack.push_back(cellitA->GetObj());
					numObj++;
				}

				for(unsigned int i = 0; i<8; i++)
				{ 
					unsigned int nextCellIx = ix_*8+1+i;
					
					if (nextCellIx >= m_numCell )
					{
						break;
					}
					
					// 再帰
					GetFullCollisionVect_Recursive(nextCellIx, retVect_);
				}

				// スタックからオブジェクトを外す
				for(unsigned int i = 0; i<numObj; i++)
					m_tmpColStack.pop_back();

				return true;
			}

		public:

			///空間番号は32bit表現で上位2ビットは使わない計算なので最大10分割
			// セル数
			// 0
			// 1
			// 1+9
			// 1+9+81
			// 1+9+81+6561
			// 1+9+81+6561+43046721 164MB?でかすぎ 
			TOctreeSpace(
				unsigned int level_,
				math::t_vector3 spaceOrigin_,
				math::t_vector3 spaceSize_) throw(std::exception)
				:
				m_divisionLevel(level_), 
				m_spaceOrigin(spaceOrigin_),
				m_spaceSize(spaceSize_),
				m_cellSize(spaceSize_/1<<level_)
			{
				// セル生成 - 一気に生成
				// 必要になったらセルを生成する実装も検討したが、キャッシュミスる気がした。
				// 後、動作時のオーバヘッドでフレーム落ちする場合があった
				const int NUM_TABLE[] = {0, 1, 1+8, 1+8+64, 1+8+64+512, 1+8+64+512+4096, 1+8+64+512+32768, 1+8+64+512++32768+262144};
				m_numCell = NUM_TABLE[level_];
				m_cellArray = n_new cell_type[m_numCell];
			}

			///@brief 
			virtual ~TOctreeSpace()
			{
				RF_SAFE_DELETE_ARRAY(m_cellArray);
			}


			///@brief AABBを空間に登録
			bool RegisterAabb(sp<CAabb3> aabb_, sp<node_type > obj_)
			{
				unsigned int spaceNo;
				unsigned int moton; //ダミー - これらを返さない実装のほうがよいか？
				unsigned int level; //ダミー - これらを返さない実装のほうがよいか？
				if (false == GetSpaceIndex(aabb_, &spaceNo, &moton, &level))
				{
					return false;
				}

				// セルに登録
				obj_->SetCollisionType(node_type::CUBE);
				obj_->m_cell = &m_cellArray[spaceNo];
				m_cellArray[spaceNo].push_back(obj_);
							
				return true;
			}
			
			///@brief 点を空間に登録
			bool RegisterPoint(sp<math::t_vector3> pos_, sp<node_type> obj_)
			{
				math::t_vector3 spacePos((pos_ -  m_spaceOrigin)/m_spaceSize);
				unsigned int moton = Space2Morton(
					(int)spacePos(0),
					(int)spacePos(1),
					(int)spacePos(2));
		
				// 必ずレベル0なのでそのモートンナンバーから空間インデックスまでのオフセットは本来固定値
				const int NUM_TABLE[] = {0, 1, 1+8, 1+8+64, 1+8+64+512, 1+8+64+512+4096, 1+8+64+512+32768, 1+8+64+512++32768+262144};
				unsigned int ix = moton + NUM_TABLE[m_divisionLevel];
				
				// セル数を超えたときは中断（範囲外を指定した時になるような気がする）
				if(spaceNo >= m_numCell)
				{
					rfinfo("out of collision space\n");
					return false;
				}

				// セルに登録
				obj_->SetCollisionType(node_type::POINT);
				m_cellArray[ix].push_back(obj_);
			
				return true;
			}
			
			///@brief ノードをセルから削除
			void Remove(sp<node_type > node_)
			{
				if(node_->m_cell == NULL)
				{
					assert(false);
					return;
				}

				// 探して削除
				cell_type::iterator it = std::find(node_->m_cell->begin(), node_->m_cell->end(), node_);
				node_->m_cell->erase(it);

				// 所属をやめる
				node_->m_cell = NULL;
			}
			
			///@brief ここに渡すvectorはできる限り再利用すること
			bool GetCollisionPairs(std::set<std::pair<sp<T>, sp<T> > >* result_)
			{
				result_->clear();			
				m_tmpColStack.clear();

				return GetFullCollisionVect_Recursive(0, result_);
			}
		};
	}
}
#endif