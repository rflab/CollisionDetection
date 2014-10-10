// ���`4���؋�ԕ��� for Collision Check
// �쐬�ҁ@	Iwahara Hiroaki
// 071026	�쐬

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
		// �I�u�W�F�N�g�R���e�i�p���X�g�m�[�h
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
			// �o�^���
			t_cell_type* m_cell;

			// ���̃m�[�h�������C�e���[�^
			// std::list�ŃR����������Ă��邩�͖��m�F�Ȃ̂�
			typename t_cell_type::iterator m_thisIs;
			
			// ����ΏۃI�u�W�F�N�g
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
		
		// ���X�g�ŏ\���Ȃ̂ō폜
		#if 0
			// CQuadTreeSpace32�Ɏg���Ă��炤
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

					// �C�e���[�^��ʂ��āANode�̃����o�ɃA�N�Z�X���邽�߂ɍ쐬
					// 	������*�Ŏ��̉����Ă������ق����������Ȃ����ߍ폜
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

					// �|�C���^�Ŕ���
					bool operator != (const iterator& rhs_)
					{
						return m_obj != rhs_.m_obj;
					}

					// ��u
					iterator& operator++(int)
					{
						return Next();
					}
				
					// �O�u
					iterator& operator++()
					{
						return Next();
					}

					// arrow
					CQuadTreeNode<T>*	operator->()
					{
						return m_obj;
					}

					// ���̉�
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
				bool			push_back(CQuadTreeNode<T>* obj_);	// �v�b�V��stl�ɍ��킹�Ă݂��B�t�ɂ�₱��������
			
			protected:
				iterator	m_begin;
				iterator	m_end;

			public:
				CQuadTreeCell();
				virtual ~CQuadTreeCell(){}

			};
		#endif
		
		// �Փˋ��
		// �N���C�A���g�͊�{�I�ɂ�����g��
		// Init()����
		// RegisterRect()����
		// GetFullCollisionVect()�ŏՓ˃y�A���擾
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
			
			// ��ԕ������x��
			unsigned int m_level;			
						 
			// �̈�̕�
			float m_regionW;			
				 
			// �̈�̍���
			float m_regionH;			
				 
			// �̈�̍��[
			float m_regionL;			
				 
			// �̈�̉��[
			float m_regionB;			
				 
			// �ŏ��Z���̕�
			float m_cellW;			
				 
			// �ŏ��Z���̍���
			float m_cellH;			
			
			// ��Ԃ̐�				 
			unsigned int m_numCell;

			// ��ԃZ���ւ̃|�C���^�z��
			t_cell_type** tm_ppCell;

			// �ՓˊK�w�T���p�f�b�N
			std::deque<T*> m_tmpColStack;
			
			// ��Ԃ̏�����
			void Init(unsigned int level_, float left_, float top_, float right_, float bottom_);
			
			/// (x_,y_)�Ԗڂ̓_�̃��[�g���ԍ�
			bool GetSpaceIndex(float left_, float top_, float right_, float bottom_, unsigned int* pulIx_, unsigned int* pulMoton_, unsigned int* pulLevel_);	// ��`�̏��������Ԕԍ�
		
			// �Փ˔�������s���āA�Փ˃y�A���擾����ċA�֐�
			bool GetFullCollisionVect_Recursive(unsigned int ix_, std::vector<T*>* retVect_);

			// �X�y�[�X���ɃZ�����m�ۂ���
			bool CreateCell(unsigned int spaceNo_);
		
		public:

			// ���W�n�͓񎟌�����
			//
			// 		y+
			// 		��
			// 		���@�E(left, top)
			// 		��
			// 		���@�@�@�E(right, bottom)
			// 		���@�@�@
			// 		������������ x+
			//
			CQuadTreeSpace32(unsigned int level_, float left_, float top_, float right_, float bottom_);
			
			virtual ~CQuadTreeSpace32();

			int RegisterRect(float left_, float top_, float right_, float bottom_, CQuadTreeNode<T>* obj_);
			
			int RegisterPoint(float x_, float y_, CQuadTreeNode<T>* obj_);
		
			// �����ɓn��vector�͂ł������ė��p���邱��
			bool GetFullCollisionVect(std::vector<T*>* retVect_);
			
			unsigned int Space2Morton(int x_, int y_);		
			
			bool Morton2Space(unsigned int moton_, int* x_, int* y_);		
		};
	
		
		/*********************************************************************************************************************/
		// ������
		/*********************************************************************************************************************/


		// �������̂�
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
			// ��Ԑ��̌v�Z(4^level)/(4-1)
			const int BIQUADRATIC[] = {1, 4, 16, 64, 256, 1024, 4096, 16384, 65536, 262144};
			m_numCell = (BIQUADRATIC[level_ + 1]-1)/3;
			
			// �|�C���^�z�񂾂����B���ꂼ��̎��Ԃ͕K�v�ɂȂ����琶������
			m_ppCell = n_new (t_cell_type*)[m_numCell];

			memset(m_ppCell, 0, sizeof(t_cell_type*)*m_numCell);

			m_regionL	= left_;
			m_regionB	= bottom_;
			m_regionW	= right_ - left_;
			m_regionH	= top_ - bottom_;
			m_cellW		= m_regionW/(1<<level_);		// 2�̏搔���V�t�g�ł�肽�������炵��
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

		// GetMortonNumber�Ɏg���Ă��炤�r�b�g���Z
		// 	8bit�V�t�g���Č��Ƙ_���a���Ƃ�}�X�N
		// 	�����4�r�b�g�V�t�g���Č��Ƙ_���a���Ƃ�}�X�N
		// 	�����2�r�b�g�V�t�g���Č��Ƙ_���a���Ƃ�}�X�N
		// 	�����1�r�b�g�V�t�g���Č��Ƙ_���a���Ƃ�}�X�N
		// 	�����32�r�b�g�̃��[�g�����������߂邽�߂�1�r�b�g���̃r�b�g�ɂȂ�
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
		// (x_,y_)�Ԗڂ̓_�̃��[�g���ԍ�
		// 	0?0? 0?0? 0?0? 0?0? 0?0? 0?0? 0?0? 0?0?
		// 	?0?0 ?0?0 ?0?0 ?0?0 ?0?0 ?0?0 ?0?0 ?0?0
		// 	�̘_���a���Ƃ�Η~����32�r�b�g�l�������ł���
		template<class T>
		unsigned int CQuadTreeSpace32<T>::Space2Morton(int x_, int y_)
		{
			// GetMortonNumber�Ɏg���Ă��炤�r�b�g���Z
			// 	8bit�V�t�g���Č��Ƙ_���a���Ƃ�}�X�N
			// 	�����4�r�b�g�V�t�g���Č��Ƙ_���a���Ƃ�}�X�N
			// 	�����2�r�b�g�V�t�g���Č��Ƙ_���a���Ƃ�}�X�N
			// 	�����1�r�b�g�V�t�g���Č��Ƙ_���a���Ƃ�}�X�N
			// 	�����32�r�b�g�̃��[�g�����������߂邽�߂�1�r�b�g���̃r�b�g�ɂȂ�

			unsigned int x = x_;
			x = (x|(x<<8)) & 0x00ff00ff;		// 0000 0000 1111 1111 0000 0000 1111 1111	��
			x = (x|(x<<4)) & 0x0f0f0f0f;		// 0000 1111 0000 1111 0000 1111 0000 1111	��
			x = (x|(x<<2)) & 0x33333333;		// 0011 0011 0011 0011 0011 0011 0011 0011	��
			x = (x|(x<<1)) & 0x55555555;		// 0101 0101 0101 0101 0101 0101 0101 0101

			unsigned int y = y_;
			y = (y|(y<<8)) & 0x00ff00ff;		// 0000 0000 1111 1111 0000 0000 1111 1111	��
			y = (y|(y<<4)) & 0x0f0f0f0f;		// 0000 1111 0000 1111 0000 1111 0000 1111	��
			y = (y|(y<<2)) & 0x33333333;		// 0011 0011 0011 0011 0011 0011 0011 0011	��
			y = (y|(y<<1)) & 0x55555555;		// 0101 0101 0101 0101 0101 0101 0101 0101

			return x | (y<<1);
		}

		/****************************************/
		// ��`�̏��������Ԕԍ�
		// ���̕ӂ͂قƂ�ǃR�s�y�ł���
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
				// ���[�g���i���o�[�̎擾
				//

				// �r���I�_���a�ŋ�Ԕԍ����������o��
				unsigned int xor = lb ^ rt;

				// (m_level-1)*2�����V�t�g������Ԃ���J�n
				unsigned int lvl = m_level;
			
				// ��Ԕԍ��v�Z(���Ƃ��Ε������x��3��0,1,2,3���x���ɂ���)
				// ���x����0����ԍׂ������
				if (0 == xor)
				{
					// �ŉ���(���x��maxi==m_level)��Xor��0�ɂȂ�A����ȍ~�Ə������قȂ�̂Ő�ɔ���
					lvl = 0;
				}
				else
				{
					// ���x��maxi-1�ȍ~
					for(; lvl > 0; lvl--)
					{
						// XOR��0�łȂ��ŏ��2�r�b�g�̑g�ݍ��킹�̏ꏊ��������
						uint32_t checkBits = (xor >> ((lvl-1)*2L)) & 0x3;
						if( checkBits != 0 )
						{
							break;
						}
					}
				}
				unsigned int motonNum = lb >> (lvl*2L);		// rb�ł�lt�ł����ʂ͓����B

				// �l��v�Z�p�̃X�^�e�B�b�N�����o
				static const int BIQUADRATIC[] = {1, 4, 16, 64, 256, 1024, 4096, 16384, 65536, 262144};
				
				// ���[�g���i���o�[����z��̎Q�Ƃ����邽�߁A�������̊K�w�ɑ��݂�����Z���̐��𑫂�
				// ����4�̓��䐔��̘a
				unsigned int ixoffset = (BIQUADRATIC[m_level-lvl]-1)/(4-1);	

				// ����4�̓��䐔��̘a�𑫂�
				unsigned int ix = motonNum + ixoffset;		

				// �Z�����𒴂����Ƃ��͓o�^���Ȃ�
				// �i�͈͊O���w�肵�����ɂȂ�悤�ȋC������j
				if(ix >= m_numCell)			
				{
					*pulLevel_ = 0x0;
					*pulMoton_ = 0x0;
					*pulIx_ = 0x0;
				
					rfinfo("�R���W�����͈͂𒴂��܂���\n");
					return false;		
				}

				*pulLevel_ = lvl;
				*pulMoton_ = motonNum;
				*pulIx_ = ix;
			#else
				unsigned int xor = lt ^ rb;	// �r���I�_���a�ŋ�Ԕԍ����������o��
				unsigned int i;
				unsigned int numShift = 1L;
				for(i=0; i < m_level; i++)
				{
					DWORD checkBits = (xor>>(i*2)) & 0x3;	// XOR��0�łȂ��ŏ��2�r�b�g�̑g�ݍ��킹�̏ꏊ��������
					if (checkBits != 0)
						numShift = i+1;
				}	
				unsigned int spaceNum = rb >> (numShift * 2L);		// rb�ł�lt�ł����ʂ͓����B
				// �����܂łŃ��[�g���i���o�[

				unsigned int addNum = (BIQUADRATIC[m_level-numShift]-1)/3;	// ����4�̓��䐔��̘a
				spaceNum += addNum;											// �𑫂�

				if(spaceNum > m_numCell)	// �Z�����𒴂����Ƃ��i�͈͊O���w�肵�����ɂȂ�悤�ȋC������j
					return 0xffffffff;		// �Ō�̃Z���ɂȂ�
			
				*pulLevel_ = lvl;
				*pulMoton_ = motonNum;
				*pulIx_ = ix;
			#endif

			return true;
		}

		// AABB����Ԃɓo�^
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
				//// if(spaceNo < m_dwCellNum ){	// ���̃`�F�b�N��GetSpaceIndex�ł��łɂ��Ă���
				// if(spaceNo >= m_numCell)
				// {
				// 	RFASSERT(false, NULL);
				// 	return false
				// }
			#endif

			// �Z�����Ȃ���΍��
			// �e�K�w���Ȃ���΍��K�v�����邽�߁A�֐����Ăяo��
			if(NULL == m_ppCell[spaceNo])
			{	
				if(false == CreateCell(spaceNo))
					return level;
			}

			// �Z���ɓo�^
			obj_->m_spaceIx    = spaceNo;
			obj_->m_spaceMoton = moton;
			obj_->m_spaceLevel = level;
			m_ppCell[spaceNo]->push_back(obj_);

			t_cell_type::iterator it = m_ppCell[spaceNo].end();
			it--;
			m_ppCell[spaceNo]->m_thisIs = it;
			
			return level;
		}


		// �_����Ԃɓo�^
		template<class T>
		int CQuadTreeSpace32<T>::RegisterPoint(
			float x_, float y_,
			CQuadTreeNode<T>* obj_)
		{
			unsigned int motonNum = Space2Morton((int)((x_ - m_regionL)/m_cellW), (int)((y_ - m_regionB)/m_cellH));
		
			// �K�����x��0�Ȃ̂ł��̃��[�g���i���o�[�����ԃC���f�b�N�X�܂ł̃I�t�Z�b�g�͖{���Œ�l
			// �l��v�Z�p�̃X�^�e�B�b�N�����o
			static const int BIQUADRATIC[] = {1, 4, 16, 64, 256, 1024, 4096, 16384, 65536, 262144};
			unsigned int ixoffset = (BIQUADRATIC[m_level]-1)/(4-1);	// ����4�̓��䐔��̘a
			unsigned int spaceNo = motonNum + ixoffset;				// ����4�̓��䐔��̘a�𑫂�

			// �Z�����𒴂����Ƃ��͒��f�i�͈͊O���w�肵�����ɂȂ�悤�ȋC������j
			if(spaceNo >= m_numCell)
			{	
				obj_->m_spaceIx    = 0x0;
				obj_->m_spaceMoton = 0x0;
				obj_->m_spaceLevel = 0x0;
			
				rfinfo("�R���W�����͈͂𒴂��܂���\n");
				return false;		// �o�^���Ȃ�
			}

			// �Z�����Ȃ���΍��
			// �e�K�w���Ȃ���΍��K�v�����邽�߁A�֐����Ăяo��
			if(NULL == m_ppCell[spaceNo])
			{	
				if(false == CreateCell(spaceNo))
					return 0;
			}

			// �Z���ɓo�^
			obj_->m_spaceIx    = spaceNo;
			obj_->m_spaceMoton = motonNum;
			obj_->m_spaceLevel = 0;
			m_ppCell[spaceNo]->push_back(obj_);
			
			t_cell_type::iterator it = m_ppCell[spaceNo].end();
			it--;
			m_ppCell[spaceNo]->m_thisIs = it;
			return 0;
		}
		

		// ���߂ēo�^������Ԃ̍쐬
		// �e�K�w���Ȃ���΍��K�v�����邽�߂��̊֐����K�v
		template<class T>
		bool CQuadTreeSpace32<T>::CreateCell(unsigned int spaceNo_)
		{
			for(;0 == m_ppCell[spaceNo_];)
			{
				m_ppCell[spaceNo_] = n_new t_cell_type;
				spaceNo_ = (spaceNo_-1)>>2;	// �e��� = (�q���-1)/4
				if(spaceNo_>=m_numCell)		// unsigned int �Ȃ̂�0�����͑傫�Ȑ����ɖ߂�
					return true;
			}
			return true;
		}

		/****************************************/
		// �Փ˔��肷��g�ݍ��킹�z��vect��Ԃ�
		// �z���2���ɂȂ��Ă邱�Ƃɒ���
		// retVec[0]��retVec[1]
		// retVec[2]��retVec[3]
		// retVec[4]��retVec[5]�c
		// �Ƃ�����ɏ��Ԃɂ��
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
		// GetFullCollisionVect�ɌĂ΂��ċA����
		template<class T>
		bool CQuadTreeSpace32<T>::GetFullCollisionVect_Recursive(
			unsigned int ix_,
			std::vector<T*>* retVect_)
		{
			// �Փ˔���y�A���X�g�C�e���[�^
			std::deque<T*>::iterator it;

			// �R���W�����Z������p�C�e���[�^
			t_cell_type::iterator cellitA = (m_ppCell[ix_])->Begin();

			// ������
			for (;cellitA != m_ppCell[ix_]->End(); cellitA++)
			{
				t_cell_type::iterator cellitB = cellitA;
				cellitB++;
				// ������Ԃ̕����Փˑg�ݍ��킹�z��ɒǉ�
				for (;cellitB != m_ppCell[ix_]->End();cellitB++)
				{
					retVect_->push_back(cellitA->GetObj());
					retVect_->push_back(cellitB->GetObj());
				}

				// �X�^�b�N�̕����Փˑg�ݍ��킹�z��ɒǉ�
				for(it=m_tmpColStack.begin(); it!=m_tmpColStack.end(); it++)
				{
					retVect_->push_back(cellitA->GetObj());
					retVect_->push_back(*it);
				}
			}

			// �q���
			bool bChildFlag = false;	// �q��Ԃ֍ċN�������H
			unsigned int numObj = 0;
			for(unsigned int i = 0; i<4; i++)
			{ 
				unsigned int nextCellIx = ix_*4+1+i;
				if (nextCellIx >= m_numCell )
				{
					break;
				}

				// nextCellIx�Ɏq��Ԃ��Ȃ�
				if (NULL == m_ppCell[nextCellIx])
				{
					continue;
				}		

				// �q��Ԃ�����Ȃ�A���񂾂������Ă���̂��X�^�b�N�ɋl�ߍ���
				// �����ō\�z�����X�^�b�N���ċA�ŗ��p����
				if(!bChildFlag)
				{
					bChildFlag = true;
					for (cellitA = (m_ppCell[ix_])->Begin(); cellitA != m_ppCell[ix_]->End(); cellitA++)
					{
						m_tmpColStack.push_back(cellitA->GetObj());
						numObj++;
					}
				}

				// �ċA
				GetFullCollisionVect_Recursive(nextCellIx, retVect_);
			}

			// �����q��Ԃ��Ȃ��Ȃ�A�X�^�b�N����I�u�W�F�N�g���O��
			// ��x�ł��X�^�b�N�ɏՓ˃I�u�W�F�N�g��o�^�����̂Ȃ�
			if(bChildFlag)
			{
				for(unsigned int i = 0; i<numObj; i++)
					m_tmpColStack.pop_back();
			}

			return true;
		}

		/********************************************************************************/
		// ������
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
		// ���X�g����O��
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
		//// ���X�g�̎��v�f
		// template<class T>
		// CQuadTreeNode<T>* CQuadTreeNode<T>::Next()
		// {
		// 	return m_next;
		// }
		//
		/// ****************************************/
		//// ���X�g�̑O�v�f
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
			// dummy�͎g�p���Ȃ����ƂƂ���
			// �ŏ��̃I�u�W�F�N�g�i�_�~�[�I�u�W�F�N�g�j��Ԃ�
			template<class T>
			sp<CQuadTreeNode<T> >& CQuadTreeCell<T>::GetBegin
			{
				return m_begin;
			}

			/****************************************/
			// �I�u�W�F�N�g�̒ǉ�
			template<class T>
			bool CQuadTreeCell<T>::push_back(CQuadTreeNode<T>* obj_)	// �v�b�V��stl�ɍ��킹�Ă݂��B�t�ɂ�₱��������
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
			// �I�u�W�F�N�g�̒ǉ�
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
			// GetFullCollisionVect�ɌĂ΂��ċA����
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
					// ������Ԃ̕����Փˑg�ݍ��킹�z��ɒǉ�
					while(obj2 != m_ppCell[elem_]->Begin)
					{
						retVect_.push_back(obj1->m_obj);
						retVect_.push_back(obj2->m_obj);
						obj2 = obj2->m_spNext;
					}
					// �X�^�b�N�̕����Փˑg�ݍ��킹�z��ɒǉ�
					for(it=colStack_.begin(); it!=colStack_.end(); it++){
						retVect_.push_back(obj1->m_obj);
						retVect_.push_back(*it);
					}
					obj1 = obj1->m_spNext;
				}

				// �q���
				bool bChildFlag = false;	// �q��Ԃ֍ċN�������H
				unsigned int numObj = 0;
				for(unsigned int i = 0; i<4; i++)
				{ 
					unsigned int nextElem = elem_*4+1+i;
					if(nextElem < m_numCell && m_ppCell[nextElem])
					{
						if(!bChildFlag){
							// �ŏ������B�����Ă���̂��X�^�b�N�ɋl�ߍ���
							obj1 = m_ppCell[elem_]->Begin->m_spNext;
							while(obj1 != m_ppCell[elem_]->Begin)
							{
								colStack_.push_back(obj1->m_obj);
								obj1 = obj1->m_spNext;
								numObj++;
							}
						}
						bChildFlag = true;
						GetFullCollisionVect_Recursive(nextElem, colStack_, retVect_);	// �q��Ԃ֍ċA
					}
				}

				// �����q��Ԃ��Ȃ��Ȃ�A�X�^�b�N����I�u�W�F�N�g���O��
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
			// �ŏ��̋�Ԃ��擾�i�{�g���j
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
				
				unsigned int ixoffset = (BIQUADRATIC[lvl]-1)/(4-1);	// ����4�̓��䐔��̘a
				unsigned int ix = motonNum + ixoffset;											// �𑫂�
			}
			
			// �q���
			bool bChildFlag = false;	// �q��Ԃ֍ċN�������H
			unsigned int numObj = 0;
			for(unsigned int i = 0; i<4; i++)
			{ 
				unsigned int nextElem = elem_*4+1+i;
				if(nextElem < m_numCell && m_ppCell[nextElem])
				{
					if(!bChildFlag){
						// �ŏ������B�����Ă���̂��X�^�b�N�ɋl�ߍ���
						obj1 = m_ppCell[elem_]->Begin->m_spNext;
						while(obj1 != m_ppCell[elem_]->Begin)
						{
							colStack_.push_back(obj1->m_obj);
							obj1 = obj1->m_spNext;
							numObj++;
						}
					}
					bChildFlag = true;
					GetFullCollisionVect_Recursive(nextElem, colStack_, retVect_);	// �q��Ԃ֍ċA
				}
			}

			std::list<T*>::iterator it;
			CQuadTreeNode<T> &obj1 = (m_ppCell[elem_]->Begin->m_spNext);
			while(obj1.GetPtr() != m_ppCell[elem_]->Begin.GetPtr())
			{
				sp<CQuadTreeNode<T> > obj2 = obj1->m_spNext;
				// ������Ԃ̕����Փˑg�ݍ��킹�z��ɒǉ�
				while(obj2 != m_ppCell[elem_]->Begin)
				{
					retVect_.push_back(obj1->m_obj);
					retVect_.push_back(obj2->m_obj);
					obj2 = obj2->m_spNext;
				}
				// �X�^�b�N�̕����Փˑg�ݍ��킹�z��ɒǉ�
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