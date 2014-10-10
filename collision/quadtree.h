// 線形4分木空間分割 for Collision Check
// 作成者　	Iwahara Hiroaki
// 071026	作成

#ifndef _IWA_COLLISION_QUADTREE_H_
#define _IWA_COLLISION_QUADTREE_H_

#include <vector>
#include <stack>
#include <deque>
#include <list>
 // for memcpy
#include <string.h>
#include <rflib/common/index.h>


///@brief rf
namespace rf
{
	///@brief collision
	namespace collision
	{
		// オブジェクトコンテナ用リストノード
		template< class T>
		class CQuadTreeNode
		{
			#if 1
				typedef std::list<CQuadTreeNode<T> > t_cell_type;
				// template class friend CQuadTreeCell<typename T>;
			#else
				typedef CQuadTreeCell<typename T> t_cell_type;
				template class friend CQuadTreeCell<typename T>;
				template class friend CQuadTreeCell<typename T>::iterator;
			#endif

		protected:
			// 登録空間
			t_cell_type* m_cell;

			// このノードを示すイテレータ
			// std::listでコレが許可されているかは未確認なので
			typename t_cell_type::iterator m_thisIs;
			
			// 判定対象オブジェクト
			T* m_obj;

			// CQuadTreeNode<T>* m_prev;
			// CQuadTreeNode<T>* m_next;
			
			unsigned int m_spaceIx; 
			unsigned int m_spaceMoton;
			unsigned int m_spaceLevel;

		public:

			CQuadTreeNode();

			explicit CQuadTreeNode(T* obj_);

			CQuadTreeNode(const CQuadTreeNode<T>& obj_)
			{
				m_cell = obj_.m_cell;
				m_obj  = obj_.m_obj;
				// m_prev = obj_.m_prev;
				// m_next = obj_.m_next;
			}

			virtual ~CQuadTreeNode()
			{
				Remove();
			}

			CQuadTreeNode<T>& operator = (const CQuadTreeNode<T>& obj_)
			{
				m_cell = obj_.m_cell;
				m_obj  = obj_.m_obj ;
				// m_prev = obj_.m_prev;
				// m_next = obj_.m_next;
				return *this;
			}

			virtual void Remove();
			
			virtual bool IsValid(){return NULL != m_cell;}
			
			#if 0
				virtual CQuadTreeNode<T>* Next();
				virtual CQuadTreeNode<T>* Prev();
			#endif
			
			void SetObj(T* obj_){m_obj = obj_;}

			T* GetObj(){return m_obj;}
		};
		
		// リストで十分なので削除
		#if 0
			// CQuadTreeSpace32に使ってもらう
			template <class T>
			class CQuadTreeCell
			{
			protected:
					CQuadTreeNode<T>	m_dummy;
			public:
				class iterator
				{
				friend CQuadTreeCell<T>;
				protected:
					CQuadTreeNode<T>*	m_obj;

				public:
					iterator()
					{
					}
		
					iterator(const iterator& obj_)
					{
						m_obj = obj_.m_obj;
					}
				
					bool HasNext()
					{
						return m_obj->m_next != m_task->m_dummy;
					}

					iterator& Next()
					{
						this->m_obj = m_obj->m_next;
						return *this;
					}

					iterator& Prev()
					{
						m_obj = m_obj->m_prev;
						return *this;
					}

					// イテレータを通して、Nodeのメンバにアクセスするために作成
					// 	→直接*で実体化してつかったほうが混乱がないため削除
					// void Remove()
					// {
					// }

					iterator& operator = (const CQuadTreeNode<T>& rhs_)
					{
						m_obj = &rhs_;
						return *this;
					}

					iterator& operator = (const iterator& rhs_)
					{
						m_obj = rhs_.m_obj;
						return *this;
					}

					// ポインタで判定
					bool operator != (const iterator& rhs_)
					{
						return m_obj != rhs_.m_obj;
					}

					// 後置
					iterator& operator++(int)
					{
						return Next();
					}
				
					// 前置
					iterator& operator++()
					{
						return Next();
					}

					// arrow
					CQuadTreeNode<T>*	operator->()
					{
						return m_obj;
					}

					// 実体化
					CQuadTreeNode<T>&	operator*()
					{
				
						return *m_obj;
					}
				};

				iterator& Begin()
				{
					m_begin.m_obj = m_dummy.m_next;
					return m_begin;
				}
				iterator& End()
				{
					m_end.m_obj= &m_dummy;
					return m_end;
				}
				bool			push_back(CQuadTreeNode<T>* obj_);	// プッシュstlに合わせてみた。逆にややこしいかも
			
			protected:
				iterator	m_begin;
				iterator	m_end;

			public:
				CQuadTreeCell();
				virtual ~CQuadTreeCell(){}

			};
		#endif
		
		// 衝突空間
		// クライアントは基本的にこれを使う
		// Init()して
		// RegisterRect()して
		// GetFullCollisionVect()で衝突ペアを取得
		template <class T>
		class CQuadTreeSpace32
		{
			typedef std::list<CQuadTreeNode<T> > t_cell_type;

		private:
			CQuadTreeSpace32(){}
			CQuadTreeSpace32(const CQuadTreeSpace32& obj_){}

		protected:
			typedef enum{
				LEVEL = 10
			}e_define;
			
			// 空間分割レベル
			unsigned int m_level;			
						 
			// 領域の幅
			float m_regionW;			
				 
			// 領域の高さ
			float m_regionH;			
				 
			// 領域の左端
			float m_regionL;			
				 
			// 領域の下端
			float m_regionB;			
				 
			// 最小セルの幅
			float m_cellW;			
				 
			// 最小セルの高さ
			float m_cellH;			
			
			// 空間の数				 
			unsigned int m_numCell;

			// 空間セルへのポインタ配列
			t_cell_type** tm_ppCell;

			// 衝突階層探索用デック
			std::deque<T*> m_tmpColStack;
			
			// 空間の初期化
			void Init(unsigned int level_, float left_, float top_, float right_, float bottom_);
			
			/// (x_,y_)番目の点のモートン番号
			bool GetSpaceIndex(float left_, float top_, float right_, float bottom_, unsigned int* pulIx_, unsigned int* pulMoton_, unsigned int* pulLevel_);	// 矩形の所属する空間番号
		
			// 衝突判定を実行して、衝突ペアを取得する再帰関数
			bool GetFullCollisionVect_Recursive(unsigned int ix_, std::vector<T*>* retVect_);

			// スペース内にセルを確保する
			bool CreateCell(unsigned int spaceNo_);
		
		public:

			// 座標系は二次元平面
			//
			// 		y+
			// 		↑
			// 		│　・(left, top)
			// 		│
			// 		│　　　・(right, bottom)
			// 		│　　　
			// 		┼────→ x+
			//
			CQuadTreeSpace32(unsigned int level_, float left_, float top_, float right_, float bottom_);
			
			virtual ~CQuadTreeSpace32();

			int RegisterRect(float left_, float top_, float right_, float bottom_, CQuadTreeNode<T>* obj_);
			
			int RegisterPoint(float x_, float y_, CQuadTreeNode<T>* obj_);
		
			// ここに渡すvectorはできる限り再利用すること
			bool GetFullCollisionVect(std::vector<T*>* retVect_);
			
			unsigned int Space2Morton(int x_, int y_);		
			
			bool Morton2Space(unsigned int moton_, int* x_, int* y_);		
		};
	
		
		/*********************************************************************************************************************/
		// 実装部
		/*********************************************************************************************************************/


		// 初期化のみ
		template<class T>
		CQuadTreeSpace32<T>::CQuadTreeSpace32(unsigned int level_, float left_, float top_, float right_, float bottom_)
			:m_level(0), m_regionW(0), m_regionH(0), m_regionL(0),
			 m_regionB(0), m_cellW(640), m_cellH(240), m_numCell(0)
		{
			Init(level_, left_, top_, right_, bottom_);
			m_tmpColStack.clear();
		}

		template<class T>
		void CQuadTreeSpace32<T>::Init(unsigned int level_, float left_, float top_, float right_, float bottom_)
		{
			// 空間数の計算(4^level)/(4-1)
			const int BIQUADRATIC[] = {1, 4, 16, 64, 256, 1024, 4096, 16384, 65536, 262144};
			m_numCell = (BIQUADRATIC[level_ + 1]-1)/3;
			
			// ポインタ配列だけ作る。それぞれの実態は必要になったら生成する
			m_ppCell = n_new (t_cell_type*)[m_numCell];

			memset(m_ppCell, 0, sizeof(t_cell_type*)*m_numCell);

			m_regionL	= left_;
			m_regionB	= bottom_;
			m_regionW	= right_ - left_;
			m_regionH	= top_ - bottom_;
			m_cellW		= m_regionW/(1<<level_);		// 2の乗数をシフトでやりたかったらしい
			m_cellH		= m_regionH/(1<<level_);
			m_level		= level_;
		}

		template<class T>
		CQuadTreeSpace32<T>::~CQuadTreeSpace32()
		{
			unsigned int i = 0;
			for(;i<m_numCell;i++)
			{
				RF_SAFE_DELETE(m_ppCell[i]);
			}
			RF_SAFE_DELETE_ARRAY(m_ppCell);
		}

		// GetMortonNumberに使ってもらうビット演算
		// 	8bitシフトして元と論理和をとりマスク
		// 	それを4ビットシフトして元と論理和をとりマスク
		// 	それを2ビットシフトして元と論理和をとりマスク
		// 	それを1ビットシフトして元と論理和をとりマスク
		// 	これで32ビットのモートン順序を求めるための1ビットずつのビットになる
		template<class T>
		bool CQuadTreeSpace32<T>::Morton2Space(unsigned int moton_, int* x_, int* y_)
		{

			unsigned int x = moton_ & 0x55555555;
			x   = ((x|(x>>1)) & 0x33333333);
			x   = ((x|(x>>2)) & 0x0f0f0f0f);
			x   = ((x|(x>>4)) & 0x00ff00ff);
			*x_ = ((x|(x>>8)) & 0x0000ffff);
		
			unsigned int y = ((moton_>>1) & 0x55555555);
			y   = ((y|(y>>1)) & 0x33333333);
			y   = ((y|(y>>2)) & 0x0f0f0f0f);
			y   = ((y|(y>>4)) & 0x00ff00ff);
			*y_ = ((y|(y>>8)) & 0x0000ffff);

			return true;
			// return (x|(y_<<1)) & 0x55555555;	// 0101 0101 0101 0101 0101 0101 0101 0101
		}
		/****************************************/
		// (x_,y_)番目の点のモートン番号
		// 	0?0? 0?0? 0?0? 0?0? 0?0? 0?0? 0?0? 0?0?
		// 	?0?0 ?0?0 ?0?0 ?0?0 ?0?0 ?0?0 ?0?0 ?0?0
		// 	の論理和をとれば欲しい32ビット値が生成できる
		template<class T>
		unsigned int CQuadTreeSpace32<T>::Space2Morton(int x_, int y_)
		{
			// GetMortonNumberに使ってもらうビット演算
			// 	8bitシフトして元と論理和をとりマスク
			// 	それを4ビットシフトして元と論理和をとりマスク
			// 	それを2ビットシフトして元と論理和をとりマスク
			// 	それを1ビットシフトして元と論理和をとりマスク
			// 	これで32ビットのモートン順序を求めるための1ビットずつのビットになる

			unsigned int x = x_;
			x = (x|(x<<8)) & 0x00ff00ff;		// 0000 0000 1111 1111 0000 0000 1111 1111	↓
			x = (x|(x<<4)) & 0x0f0f0f0f;		// 0000 1111 0000 1111 0000 1111 0000 1111	↓
			x = (x|(x<<2)) & 0x33333333;		// 0011 0011 0011 0011 0011 0011 0011 0011	↓
			x = (x|(x<<1)) & 0x55555555;		// 0101 0101 0101 0101 0101 0101 0101 0101

			unsigned int y = y_;
			y = (y|(y<<8)) & 0x00ff00ff;		// 0000 0000 1111 1111 0000 0000 1111 1111	↓
			y = (y|(y<<4)) & 0x0f0f0f0f;		// 0000 1111 0000 1111 0000 1111 0000 1111	↓
			y = (y|(y<<2)) & 0x33333333;		// 0011 0011 0011 0011 0011 0011 0011 0011	↓
			y = (y|(y<<1)) & 0x55555555;		// 0101 0101 0101 0101 0101 0101 0101 0101

			return x | (y<<1);
		}

		/****************************************/
		// 矩形の所属する空間番号
		// この辺はほとんどコピペですな
		template<class T>
		bool CQuadTreeSpace32<T>::GetSpaceIndex(
			float left_, float top_, float right_, float bottom_,
			unsigned int* pulIx_, unsigned int* pulMoton_, unsigned int* pulLevel_)
		{
			int l = (int)((left_   - m_regionL)/m_cellW);
			int t = (int)((top_    - m_regionB)/m_cellH);
			int r = (int)((right_  - m_regionL)/m_cellW);
			int b = (int)((bottom_ - m_regionB)/m_cellH);

			unsigned int lb = Space2Morton(l, b);
			unsigned int rt = Space2Morton(r, t);

			#if 1
			
				//
				// モートンナンバーの取得
				//

				// 排他的論理和で空間番号だけ抜き出す
				unsigned int xor = lb ^ rt;

				// (m_level-1)*2だけシフトした状態から開始
				unsigned int lvl = m_level;
			
				// 空間番号計算(たとえば分割レベル3は0,1,2,3レベルについて)
				// レベルは0が一番細かい空間
				if (0 == xor)
				{
					// 最下位(レベルmaxi==m_level)はXorが0になり、それ以降と処理が異なるので先に判定
					lvl = 0;
				}
				else
				{
					// レベルmaxi-1以降
					for(; lvl > 0; lvl--)
					{
						// XORの0でない最上位2ビットの組み合わせの場所を見つける
						uint32_t checkBits = (xor >> ((lvl-1)*2L)) & 0x3;
						if( checkBits != 0 )
						{
							break;
						}
					}
				}
				unsigned int motonNum = lb >> (lvl*2L);		// rbでもltでも結果は同じ。

				// 四乗計算用のスタティックメンバ
				static const int BIQUADRATIC[] = {1, 4, 16, 64, 256, 1024, 4096, 16384, 65536, 262144};
				
				// モートンナンバーから配列の参照をするため、それより上の階層に存在しえるセルの数を足す
				// 公比4の等比数列の和
				unsigned int ixoffset = (BIQUADRATIC[m_level-lvl]-1)/(4-1);	

				// 公比4の等比数列の和を足す
				unsigned int ix = motonNum + ixoffset;		

				// セル数を超えたときは登録しない
				// （範囲外を指定した時になるような気がする）
				if(ix >= m_numCell)			
				{
					*pulLevel_ = 0x0;
					*pulMoton_ = 0x0;
					*pulIx_ = 0x0;
				
					rfinfo("コリジョン範囲を超えました\n");
					return false;		
				}

				*pulLevel_ = lvl;
				*pulMoton_ = motonNum;
				*pulIx_ = ix;
			#else
				unsigned int xor = lt ^ rb;	// 排他的論理和で空間番号だけ抜き出す
				unsigned int i;
				unsigned int numShift = 1L;
				for(i=0; i < m_level; i++)
				{
					DWORD checkBits = (xor>>(i*2)) & 0x3;	// XORの0でない最上位2ビットの組み合わせの場所を見つける
					if (checkBits != 0)
						numShift = i+1;
				}	
				unsigned int spaceNum = rb >> (numShift * 2L);		// rbでもltでも結果は同じ。
				// ここまででモートンナンバー

				unsigned int addNum = (BIQUADRATIC[m_level-numShift]-1)/3;	// 公比4の等比数列の和
				spaceNum += addNum;											// を足す

				if(spaceNum > m_numCell)	// セル数を超えたとき（範囲外を指定した時になるような気がする）
					return 0xffffffff;		// 最後のセルになる
			
				*pulLevel_ = lvl;
				*pulMoton_ = motonNum;
				*pulIx_ = ix;
			#endif

			return true;
		}

		// AABBを空間に登録
		template<class T>
		int CQuadTreeSpace32<T>::RegisterRect(
			float left_, float top_, float right_, float bottom_,
			CQuadTreeNode<T>* obj_)
		{
			unsigned int spaceNo;
			unsigned int moton;
			unsigned int level;
		
			#if 1
				if (false == GetSpaceIndex(left_, top_, right_, bottom_, &spaceNo, &moton, &level))
				{
					return false;
				}
			#else
				//// if(spaceNo < m_dwCellNum ){	// このチェックはGetSpaceIndexですでにしてある
				// if(spaceNo >= m_numCell)
				// {
				// 	RFASSERT(false, NULL);
				// 	return false
				// }
			#endif

			// セルがなければ作る
			// 親階層もなければ作る必要があるため、関数を呼び出す
			if(NULL == m_ppCell[spaceNo])
			{	
				if(false == CreateCell(spaceNo))
					return level;
			}

			// セルに登録
			obj_->m_spaceIx    = spaceNo;
			obj_->m_spaceMoton = moton;
			obj_->m_spaceLevel = level;
			m_ppCell[spaceNo]->push_back(obj_);

			t_cell_type::iterator it = m_ppCell[spaceNo].end();
			it--;
			m_ppCell[spaceNo]->m_thisIs = it;
			
			return level;
		}


		// 点を空間に登録
		template<class T>
		int CQuadTreeSpace32<T>::RegisterPoint(
			float x_, float y_,
			CQuadTreeNode<T>* obj_)
		{
			unsigned int motonNum = Space2Morton((int)((x_ - m_regionL)/m_cellW), (int)((y_ - m_regionB)/m_cellH));
		
			// 必ずレベル0なのでそのモートンナンバーから空間インデックスまでのオフセットは本来固定値
			// 四乗計算用のスタティックメンバ
			static const int BIQUADRATIC[] = {1, 4, 16, 64, 256, 1024, 4096, 16384, 65536, 262144};
			unsigned int ixoffset = (BIQUADRATIC[m_level]-1)/(4-1);	// 公比4の等比数列の和
			unsigned int spaceNo = motonNum + ixoffset;				// 公比4の等比数列の和を足す

			// セル数を超えたときは中断（範囲外を指定した時になるような気がする）
			if(spaceNo >= m_numCell)
			{	
				obj_->m_spaceIx    = 0x0;
				obj_->m_spaceMoton = 0x0;
				obj_->m_spaceLevel = 0x0;
			
				rfinfo("コリジョン範囲を超えました\n");
				return false;		// 登録しない
			}

			// セルがなければ作る
			// 親階層もなければ作る必要があるため、関数を呼び出す
			if(NULL == m_ppCell[spaceNo])
			{	
				if(false == CreateCell(spaceNo))
					return 0;
			}

			// セルに登録
			obj_->m_spaceIx    = spaceNo;
			obj_->m_spaceMoton = motonNum;
			obj_->m_spaceLevel = 0;
			m_ppCell[spaceNo]->push_back(obj_);
			
			t_cell_type::iterator it = m_ppCell[spaceNo].end();
			it--;
			m_ppCell[spaceNo]->m_thisIs = it;
			return 0;
		}
		

		// 初めて登録される空間の作成
		// 親階層もなければ作る必要があるためこの関数が必要
		template<class T>
		bool CQuadTreeSpace32<T>::CreateCell(unsigned int spaceNo_)
		{
			for(;0 == m_ppCell[spaceNo_];)
			{
				m_ppCell[spaceNo_] = n_new t_cell_type;
				spaceNo_ = (spaceNo_-1)>>2;	// 親空間 = (子空間-1)/4
				if(spaceNo_>=m_numCell)		// unsigned int なので0未満は大きな数字に戻る
					return true;
			}
			return true;
		}

		/****************************************/
		// 衝突判定する組み合わせ配列vectを返す
		// 配列は2つずつになってることに注意
		// retVec[0]とretVec[1]
		// retVec[2]とretVec[3]
		// retVec[4]とretVec[5]…
		// という具合に順番にやる
		// template<class T>
		// std::vector<T*>& CQuadTreeSpace32<T>::GetFullCollisionVect()
		//// const std::vector<T*>& CQuadTreeSpace32<T>::GetFullCollisionVect()
		// {
		// 	m_pairs.clear();
		// 	if(m_ppCell[0]==NULL)
		// 		return 0;

		// 	m_tmpColStack.clear();

		// 	GetFullCollisionVect_Recursive(0, m_tmpColStack, m_pairs);
		// 	
		// 	return m_pairs;
		// }

		template<class T>
		bool CQuadTreeSpace32<T>::GetFullCollisionVect(std::vector<T*>* retVec_)
		{
			retVec_->clear();
			if(m_ppCell[0]==NULL)
				return 0;
			
			m_tmpColStack.clear();

			return GetFullCollisionVect_Recursive(0, retVec_);
		}
		/****************************************/
		// GetFullCollisionVectに呼ばれる再帰部分
		template<class T>
		bool CQuadTreeSpace32<T>::GetFullCollisionVect_Recursive(
			unsigned int ix_,
			std::vector<T*>* retVect_)
		{
			// 衝突判定ペアリストイテレータ
			std::deque<T*>::iterator it;

			// コリジョンセル巡回用イテレータ
			t_cell_type::iterator cellitA = (m_ppCell[ix_])->Begin();

			// 同一空間
			for (;cellitA != m_ppCell[ix_]->End(); cellitA++)
			{
				t_cell_type::iterator cellitB = cellitA;
				cellitB++;
				// 同じ空間の分を衝突組み合わせ配列に追加
				for (;cellitB != m_ppCell[ix_]->End();cellitB++)
				{
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

			// 子空間
			bool bChildFlag = false;	// 子空間へ再起したか？
			unsigned int numObj = 0;
			for(unsigned int i = 0; i<4; i++)
			{ 
				unsigned int nextCellIx = ix_*4+1+i;
				if (nextCellIx >= m_numCell )
				{
					break;
				}

				// nextCellIxに子空間がない
				if (NULL == m_ppCell[nextCellIx])
				{
					continue;
				}		

				// 子空間があるなら、初回だけ持ってるものをスタックに詰め込む
				// ここで構築したスタックを再帰で流用する
				if(!bChildFlag)
				{
					bChildFlag = true;
					for (cellitA = (m_ppCell[ix_])->Begin(); cellitA != m_ppCell[ix_]->End(); cellitA++)
					{
						m_tmpColStack.push_back(cellitA->GetObj());
						numObj++;
					}
				}

				// 再帰
				GetFullCollisionVect_Recursive(nextCellIx, retVect_);
			}

			// もう子空間がないなら、スタックからオブジェクトを外す
			// 一度でもスタックに衝突オブジェクトを登録したのなら
			if(bChildFlag)
			{
				for(unsigned int i = 0; i<numObj; i++)
					m_tmpColStack.pop_back();
			}

			return true;
		}

		/********************************************************************************/
		// 初期化
		template<class T> 
		CQuadTreeNode<T>::CQuadTreeNode()
		:m_cell(NULL)
		,m_obj(NULL)
		// ,m_prev(NULL)
		// ,m_next(NULL)
		{
		}

		template<class T> 
		CQuadTreeNode<T>::CQuadTreeNode(T* obj_)
		:m_cell(NULL)
		,m_obj(obj_)
		// ,m_prev(NULL)
		// ,m_next(NULL)
		{
		}
		
		/****************************************/
		// リストから外す
		template<class T>
		void CQuadTreeNode<T>::Remove()
		{
			if(m_cell == NULL)
				return;
			// m_prev->m_next = m_next;
			// m_next->m_prev = m_prev;
			m_cell = NULL;
			// Prev()->Next() = Next();
			// Next()->Prev() = Prev();
		}

		/// ****************************************/
		//// リストの次要素
		// template<class T>
		// CQuadTreeNode<T>* CQuadTreeNode<T>::Next()
		// {
		// 	return m_next;
		// }
		//
		/// ****************************************/
		//// リストの前要素
		// template<class T>
		// CQuadTreeNode<T>* CQuadTreeNode<T>::Prev()
		// {
		// 	return m_prev;
		// }

		#if 0
			/********************************************************************************/
			template<class T>
			CQuadTreeCell<T>::CQuadTreeCell()
			{
				// m_num = 0;
				m_dummy.m_next = &m_dummy;
				m_dummy.m_prev = &m_dummy;
			}

			/****************************************
			// dummyは使用しないこととした
			// 最初のオブジェクト（ダミーオブジェクト）を返す
			template<class T>
			sp<CQuadTreeNode<T> >& CQuadTreeCell<T>::GetBegin
			{
				return m_begin;
			}

			/****************************************/
			// オブジェクトの追加
			template<class T>
			bool CQuadTreeCell<T>::push_back(CQuadTreeNode<T>* obj_)	// プッシュstlに合わせてみた。逆にややこしいかも
			{
				// m_num++;
				obj_->m_next = &m_dummy;
				obj_->m_prev = m_dummy.m_prev;
				obj_->m_cell = this;

				m_dummy.m_prev->m_next = obj_;
				m_dummy.m_prev = obj_;
		
				return true;
			}

			/****************************************
			// オブジェクトの追加
			template<class T>
			bool CQuadTreeCell<T>::push_back(sp<CQuadTreeNode<T> > obj_)
			{
				sp<CQuadTreeNode<T> > tmp = m_begin->m_spPrev;
				tmp->m_spNext = obj_;
				m_begin->m_spPrev = obj_;
				obj_->m_spNext = m_begin;
				obj_->m_spPrev = tmp;
				obj_->m_cell = this;
				return true;
			}
			/********************************************************************************
			// GetFullCollisionVectに呼ばれる再帰部分
			template<class T>
			unsigned int CQuadTreeSpace32<T>::GetFullCollisionVect_Recursive(
				unsigned int elem_,
				std::list<T*> &colStack_,
				std::vector<T*> &retVect_)
			{
				std::list<T*>::iterator it;

		
				CQuadTreeNodeIterator<T> &it = (m_ppCell[elem_]->Begin();
				while(it != m_ppCell[elem_]->Begin.GetPtr())
				{
					sp<CQuadTreeNode<T> > obj2 = obj1->m_spNext;
					// 同じ空間の分を衝突組み合わせ配列に追加
					while(obj2 != m_ppCell[elem_]->Begin)
					{
						retVect_.push_back(obj1->m_obj);
						retVect_.push_back(obj2->m_obj);
						obj2 = obj2->m_spNext;
					}
					// スタックの分を衝突組み合わせ配列に追加
					for(it=colStack_.begin(); it!=colStack_.end(); it++){
						retVect_.push_back(obj1->m_obj);
						retVect_.push_back(*it);
					}
					obj1 = obj1->m_spNext;
				}

				// 子空間
				bool bChildFlag = false;	// 子空間へ再起したか？
				unsigned int numObj = 0;
				for(unsigned int i = 0; i<4; i++)
				{ 
					unsigned int nextElem = elem_*4+1+i;
					if(nextElem < m_numCell && m_ppCell[nextElem])
					{
						if(!bChildFlag){
							// 最初だけ。持ってるものをスタックに詰め込む
							obj1 = m_ppCell[elem_]->Begin->m_spNext;
							while(obj1 != m_ppCell[elem_]->Begin)
							{
								colStack_.push_back(obj1->m_obj);
								obj1 = obj1->m_spNext;
								numObj++;
							}
						}
						bChildFlag = true;
						GetFullCollisionVect_Recursive(nextElem, colStack_, retVect_);	// 子空間へ再帰
					}
				}

				// もう子空間がないなら、スタックからオブジェクトを外す
				if(bChildFlag)
					for(unsigned int i = 0; i<numObj; i++)
						colStack_.pop_back();

				return (unsigned int)retVect_.size();
			}
			*/
		#endif
	
		/****************************************
		template<class T>
		unsigned int CQuadTreeSpace32<T>::GetCollisionArray(T** &pul_, unsigned int size_, float left_, float top_, float right_, float bottom_) const
		{
			unsigned int ix;
			unsigned int moton;
			unsigned int level;
			// 最初の空間を取得（ボトム）
			if (false != GetSpaceIndex(left_, top_, right_, bottom_, &ix, &moton, &level))
			{
				RFASSERT(false, NULL);
				return false;
			}

			for (int i=level; i>=0; i--)
			{
				if(NULL == m_ppCell[ix])
					continue;

				level>>2;
				
				unsigned int ixoffset = (BIQUADRATIC[lvl]-1)/(4-1);	// 公比4の等比数列の和
				unsigned int ix = motonNum + ixoffset;											// を足す
			}
			
			// 子空間
			bool bChildFlag = false;	// 子空間へ再起したか？
			unsigned int numObj = 0;
			for(unsigned int i = 0; i<4; i++)
			{ 
				unsigned int nextElem = elem_*4+1+i;
				if(nextElem < m_numCell && m_ppCell[nextElem])
				{
					if(!bChildFlag){
						// 最初だけ。持ってるものをスタックに詰め込む
						obj1 = m_ppCell[elem_]->Begin->m_spNext;
						while(obj1 != m_ppCell[elem_]->Begin)
						{
							colStack_.push_back(obj1->m_obj);
							obj1 = obj1->m_spNext;
							numObj++;
						}
					}
					bChildFlag = true;
					GetFullCollisionVect_Recursive(nextElem, colStack_, retVect_);	// 子空間へ再帰
				}
			}

			std::list<T*>::iterator it;
			CQuadTreeNode<T> &obj1 = (m_ppCell[elem_]->Begin->m_spNext);
			while(obj1.GetPtr() != m_ppCell[elem_]->Begin.GetPtr())
			{
				sp<CQuadTreeNode<T> > obj2 = obj1->m_spNext;
				// 同じ空間の分を衝突組み合わせ配列に追加
				while(obj2 != m_ppCell[elem_]->Begin)
				{
					retVect_.push_back(obj1->m_obj);
					retVect_.push_back(obj2->m_obj);
					obj2 = obj2->m_spNext;
				}
				// スタックの分を衝突組み合わせ配列に追加
				for(it=colStack_.begin(); it!=colStack_.end(); it++){
					retVect_.push_back(obj1->m_obj);
					retVect_.push_back(*it);
				}
				obj1 = obj1->m_spNext;
			}
			
			retVect_.clear();
			if(m_ppCell[0]==NULL)
				return 0;

			std::list<T*> colStack;

			GetFullCollisionVect_Recursive(0, colStack, retVect_);
		
			return (unsigned int)retVect_.size();

		}
		*/
	}
}
#endif