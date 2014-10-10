// �Փ˔���N���X
// rf. All Right Reserves.

// 2008/06/03�@OBB�ǉ����S����蒼��

#ifndef _RF_COLLISION_TRIANGLE_H_
#define _RF_COLLISION_TRIANGLE_H_

#include <rflib/math/index.h>

///@brief rf
namespace rf
{
	///@brief collision
	namespace collision
	{
		///�{���ɂ����̎O�p�`�B�C���f�b�N�X���͎g���Ă��Ȃ�
		template<class Vert>
		class TTriangle
		{
		public:
			///@brief ���_
			Vert vertex[3];
		};

		///�O�p�`���_�C���f�b�N�X
		template<class Vert>
		class CTriangleIndex
		{
		public:
			///@brief ���_�z��
			///boost::optional< sp<vector<Vert> > vertices;

			///@brief ���_�C���f�b�N�X
			int index[3];
		};
		
		#if 0
			// �ŏI�`��
			// ������ʃ��b�V���𔻒�ł���悤�ɂ���\��
			class CColMesh3D : public CCollision3D
			{
			private:

			public:
				float			m_radius;			// ���a			
				CTriMesh	m_IndexedPrimitive;	// ���_�z��	
				D3DXMATRIX		m_world;			// �����ϊ��s��F�{�b�N�X�̈ʒu�͒��S�_����Ƃ���
				D3DXVECTOR3		m_v;				// ���x
				D3DXVECTOR3		m_omega;			// �p���x

				CColMesh3D(const CTriMesh& mesh_, float BSphereR = 0);
				virtual ~CColMesh3D();

				virtual void SetWorld(D3DXMATRIX &world_){m_world = world_;}
				virtual const D3DXMATRIX& GetWorld(){return m_world;}

				void Draw();

				// �ȉ��̓^�C�v�Ⴂ�̃I�[�o�[���[�h��R
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
