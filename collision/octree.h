// ���`8���؋�ԕ��� Octree
// �쐬�ҁ@	Iwahara Hiroaki
// 071026	�쐬

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
		// �I�u�W�F�N�g�R���e�i�p���X�g�m�[�h
		template<class T>
		class TOctreeNode
		{
			typedef std::list<TOctreeNode<T> > cell_type;
			typedef TOctreeNode<T> self_type;
			


		protected:

			///@brief ���̃Z����o�^�������
			cell_type* m_cell;
						
			///@brief ����ΏۃI�u�W�F�N�g
			sp<T> m_obj;
			
		public:

			///@brief �R���X�g���N�^
			TOctreeNode()
				:
				m_cell(NULL),
				m_obj(NULL),
				m_type(INVALID)
			{
			}

			///@brief �R���X�g���N�^
			explicit TOctreeNode(sp<T> obj_)
				:
				m_cell(NULL),
				m_obj(obj_),
				m_type(INVALID)
			{
			}
		
			///@brief �R���X�g���N�^
			TOctreeNode(const self_type& obj_)
			{
				m_cell = obj_.m_cell;
				m_obj  = obj_.m_obj;
				m_type = obj_.m_type;
			}

			///@brief �f�X�g���N�^
			virtual ~TOctreeNode()
			{
				Remove();
			}

			///@brief ���
			self_type& operator = (const self_type& obj_)
			{
				m_cell = obj_.m_cell;
				m_obj  = obj_.m_obj;
				m_type = obj_.m_type;
				return *this;
			}
			
			///@brief �Z���ɓo�^���Ă���H
			bool IsValid()
			{
				return NULL != m_cell;
			}
			
			///@brief �Z���ɓo�^���Ă���H
			void SetCell(cell_type* cell_)
			{
				m_cell = cell_;
			}
			
			///@brief �o�^��̃Z�����擾����
			cell_type* GetCell()
			{
				return m_cell;
			}
			
			///@brief �I�u�W�F�N�g�o�^
			void SetObj(sp<T> obj_)
			{
				m_obj = obj_;
			}

			///@brief �o�^����Ă���I�u�W�F�N�g��Ԃ�
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
		

		///@brief �Փˋ��
		template <class T>
		class TOctreeSpace
		{
			
			///@brief �o�^����Z���̃^�C�v
			typedef TOctreeNode<T> node_type;

			///@brief �o�^����Z���̃^�C�v
			typedef std::list<sp<node_type> > cell_type;
											
			///@brief �Փ˃^�C�v
			typedef enum			
			{
				POINT,
				CUBE,
				INVALID
			}collision_type;
			
			///@brief �Փ˃^�C�v
			collision_type m_type;

			///@brief �Փ˃^�C�v��ݒ肷��
			void SetCollisionType(collision_type type_)
			{
				m_type = type_;
			}

			///@brief �Փ˃^�C�v���擾����
			collision_type GetCollisionType()
			{
				return m_type;
			}

		private:
			
			///@brief �R���X�g���N�^�}�~
			TOctreeSpace(){}
			
			///@brief �R�s�[�R���X�g���N�^�}�~
			TOctreeSpace(const TOctreeSpace& obj_){}

		protected:
			
			///@brief ��ԕ����̐[���i�������x���j
			unsigned int m_divisionLevel;			
			
			///@brief �̈�̕��T�C�Y
			math::t_vector3 m_spaceSize;

			///@brief �̈�̒[�̈ʒu
			math::t_vector3 m_spaceOrigin;

			///@brief �ŏ��Z���̕�
			math::t_vector3 m_cellSize;			
				 
			///@brief ��Ԃ̐�				 
			unsigned int m_numCell;

			///@brief ��ԃZ���ւ̃|�C���^�z��
			cell_type* m_cellArray;

			///@brief �ՓˊK�w�T���p�f�b�N
			std::deque<T*> m_tmpColStack;
			
		protected:
			
			///@brief ���[�g���ԍ�����Z���z��C���f�b�N�X�������Ɏ擾����
			///@note 
			// (x_,y_,z_)�Ԗڂ̓_�̃��[�g���ԍ�
			// 	000? 00?0 00?0 0?00 ?00? 00?0 0?00 ?00?
			// 	00?0 0?00 ?00? 00?0 0?00 0?00 ?00? 00?0
			// 	0?00 ?00? 00?0 0?00 ?000 ?00? 00?0 0?00
			unsigned int Space2Morton(
				unsigned int x_,
				unsigned int y_,
				unsigned int z_)
			{
				unsigned int x = x_;
				x = (x|(x<<8)) & 0x0000f00f; // 0000 xxxx 0000 0000 1111 0000 0000 1111 ��
				x = (x|(x<<4)) & 0x000c30c3; // xx00 00xx 0000 1100 0011 0000 1100 0011 ��
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

			///@brief �Z���z��C���f�b�N�X���烂�[�g���ԍ��������Ɏ擾����
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

			///@brief �o�E���f�B���O�{�b�N�X�̏��������Ԕԍ�
			bool GetSpaceIndex(
				const CAabb3& aabb_,
				unsigned int* ix_,
				unsigned int* moton_,
				unsigned int* level_)
			{
				// �o�E���f�B���O�{�b�N�X�̍ŏ��E�ő�
				int xl = (int)((aabb_.mini(0) - m_regionOrigin(0)) / m_cellSize(0));
				int yl = (int)((aabb_.mini(0) - m_regionOrigin(1)) / m_cellSize(1));
				int zl = (int)((aabb_.mini(0) - m_regionOrigin(2)) / m_cellSize(2));
				int xm = (int)((aabb_.maxi(0) - m_regionOrigin(0)) / m_cellSize(0));
				int ym = (int)((aabb_.maxi(0) - m_regionOrigin(1)) / m_cellSize(1));
				int zm = (int)((aabb_.maxi(0) - m_regionOrigin(2)) / m_cellSize(2));

				// ���[�g���i���o�[�̎擾
				unsigned int minMoton = Space2Morton(xl, yl, zl);
				unsigned int maxMoton = Space2Morton(xm, ym, zm);
						
				// �e��Ԕԍ����������o��
				// �e��ԕ��������ʂȏꍇ�A���̕����͔r���I�_���a��0�ɂȂ�
				// ���Ƃ��Δr���I�_���ւ�0xffffff00�Ƃ̘_���ς�0�Ȃ�����ƍ����x��
				// TODO �r�b�g���Z�Ƃ��ō����Ɉꔭ�Ōv�Z������@�����肻���ȋC�����邪�v�����Ȃ������B�i����Z�K�{������j
				unsigned int xorMoton = l ^ m;
				assert(xorMoton & ~((0x1 << (m_divisionLevel*3))-1) != 0 && "�͈͊O");
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
								
				// ���[�g���i���o�[����z��̃C���f�b�N�X�A
				// �������̊K�w�ɑ��݂�����Z���̐��𑫂����߂̃e�[�u��
				// ����9�̓��䐔��̘a(8^n-1)/(8-1)
				const int OFFSETS[] = {0, 1, 1+8, 1+8+64, 1+8+64+512, 1+8+64+512+4096, 1+8+64+512+32768, 1+8+64+512++32768+262144};
				unsigned int ix = parentMoton + OFFSETS[level];		

				*level_ = level;
				*moton_ = motonNum;
				*ix_ = ix;
				return true;
			}

			///@brief �Փ˔�������s���āA�Փ˃y�A���擾����ċA�֐�
			bool GetFullCollisionVect_Recursive(unsigned int ix_, std::set<std::pair<sp<T>, sp<T> > >* retVect_)
			{
				// �Փ˔���y�A���X�g�C�e���[�^
				std::deque<T*>::iterator it;

				// �R���W�����Z������p�C�e���[�^
				cell_type::iterator cellitA = (m_cellArray[ix_])->begin();

				// ������
				for (;cellitA != m_cellArray[ix_]->end(); cellitA++)
				{
					cell_type::iterator cellitB = cellitA;
					cellitB++;
					
					// ������Ԃ̕����Փˑg�ݍ��킹�z��ɒǉ�
					for (;cellitB != m_cellArray[ix_]->end();cellitB++)
					{
						//
						rfinfo("�o�^���Փ˔��肵����");

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

				
				// �����Ă���̂��X�^�b�N�ɋl�ߍ���
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
					
					// �ċA
					GetFullCollisionVect_Recursive(nextCellIx, retVect_);
				}

				// �X�^�b�N����I�u�W�F�N�g���O��
				for(unsigned int i = 0; i<numObj; i++)
					m_tmpColStack.pop_back();

				return true;
			}

		public:

			///��Ԕԍ���32bit�\���ŏ��2�r�b�g�͎g��Ȃ��v�Z�Ȃ̂ōő�10����
			// �Z����
			// 0
			// 1
			// 1+9
			// 1+9+81
			// 1+9+81+6561
			// 1+9+81+6561+43046721 164MB?�ł����� 
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
				// �Z������ - ��C�ɐ���
				// �K�v�ɂȂ�����Z���𐶐���������������������A�L���b�V���~�X��C�������B
				// ��A���쎞�̃I�[�o�w�b�h�Ńt���[����������ꍇ��������
				const int NUM_TABLE[] = {0, 1, 1+8, 1+8+64, 1+8+64+512, 1+8+64+512+4096, 1+8+64+512+32768, 1+8+64+512++32768+262144};
				m_numCell = NUM_TABLE[level_];
				m_cellArray = n_new cell_type[m_numCell];
			}

			///@brief 
			virtual ~TOctreeSpace()
			{
				RF_SAFE_DELETE_ARRAY(m_cellArray);
			}


			///@brief AABB����Ԃɓo�^
			bool RegisterAabb(sp<CAabb3> aabb_, sp<node_type > obj_)
			{
				unsigned int spaceNo;
				unsigned int moton; //�_�~�[ - ������Ԃ��Ȃ������̂ق����悢���H
				unsigned int level; //�_�~�[ - ������Ԃ��Ȃ������̂ق����悢���H
				if (false == GetSpaceIndex(aabb_, &spaceNo, &moton, &level))
				{
					return false;
				}

				// �Z���ɓo�^
				obj_->SetCollisionType(node_type::CUBE);
				obj_->m_cell = &m_cellArray[spaceNo];
				m_cellArray[spaceNo].push_back(obj_);
							
				return true;
			}
			
			///@brief �_����Ԃɓo�^
			bool RegisterPoint(sp<math::t_vector3> pos_, sp<node_type> obj_)
			{
				math::t_vector3 spacePos((pos_ -  m_spaceOrigin)/m_spaceSize);
				unsigned int moton = Space2Morton(
					(int)spacePos(0),
					(int)spacePos(1),
					(int)spacePos(2));
		
				// �K�����x��0�Ȃ̂ł��̃��[�g���i���o�[�����ԃC���f�b�N�X�܂ł̃I�t�Z�b�g�͖{���Œ�l
				const int NUM_TABLE[] = {0, 1, 1+8, 1+8+64, 1+8+64+512, 1+8+64+512+4096, 1+8+64+512+32768, 1+8+64+512++32768+262144};
				unsigned int ix = moton + NUM_TABLE[m_divisionLevel];
				
				// �Z�����𒴂����Ƃ��͒��f�i�͈͊O���w�肵�����ɂȂ�悤�ȋC������j
				if(spaceNo >= m_numCell)
				{
					rfinfo("out of collision space\n");
					return false;
				}

				// �Z���ɓo�^
				obj_->SetCollisionType(node_type::POINT);
				m_cellArray[ix].push_back(obj_);
			
				return true;
			}
			
			///@brief �m�[�h���Z������폜
			void Remove(sp<node_type > node_)
			{
				if(node_->m_cell == NULL)
				{
					assert(false);
					return;
				}

				// �T���č폜
				cell_type::iterator it = std::find(node_->m_cell->begin(), node_->m_cell->end(), node_);
				node_->m_cell->erase(it);

				// ��������߂�
				node_->m_cell = NULL;
			}
			
			///@brief �����ɓn��vector�͂ł������ė��p���邱��
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