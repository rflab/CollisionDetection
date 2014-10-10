/// rflab. All Rights Reserved.
///@file	collisiondetectfunction.cpp
///@auther	rflab.
///@date	2010/06/24
///@brief	衝突判定関数

#include <bighead.h>
#include <rflib/common/index.h>
#include "collisiondetectfunction.h"
#include <cassert>
#include <algorithm>


using namespace rf;
using namespace collision;
using namespace math;


namespace rf
{
	namespace collision
	{
		// ------------------------------------------------------------------
		// 2d
		// ------------------------------------------------------------------

		///@brief	点-点の距離を求める
		///@param	a_	点Aの座標
		///@param   b_	点Bの座標
		///@retval  点-点の距離
		float PtPtDistance(
			const t_vector2&	a_,
			const t_vector2&	b_)
		{
			t_vector2 ab(b_ - a_);
			return ab.Length();
		}

		///@brief	円-円の衝突を判定する
		///@param	centerA_	円Aの座標
		///@param	radiusA_	円Aの座標
		///@param   centerB_	円Bの座標
		///@param   radiusB_	円Bの座標
		///@note
		///@retval  点-点の距離
		float CircleCircleCol(
			const t_vector2&	centerA_,
			float 				radiusA_,
			const t_vector2&	centerB_,
			float 				radiusB_)
		{
			t_vector2 ab(centerA_ - centerB_);
			return ab.Length() < radiusA_ + radiusB_;
		}

		///@brief	AABB-点の衝突
		///@param	aabb_	AABB
		///@param	pt_		点の座標
		///@retval	true	衝突あり
		///@retval	false	衝突なし
		bool AabbPtCol(
			const CAabb2&			aabb_,
			const math::t_vector2&	pt_)
		{
			return	pt_(0) <= aabb_.mini(0) && pt_(0) <= aabb_.mini(0) &&
					pt_(1) <= aabb_.mini(1) && pt_(1) <= aabb_.mini(1);
		}

		///@brief	AABB-点最近接点を取得する
		///@param	aabb_			AABB
		///@param	p_				点の座標
		///@param	proximate_		[out]最近接点
		void ::rf::collision::AabbPtProximate(
			const CAabb2&			aabb_,	
			const math::t_vector2&	p_,
			math::t_vector2*		proximate_)
		{
			for (int i=0; i<2; i++)
			{
				if (p_(i) < aabb_.mini(i))
					(*proximate_)(i) = aabb_.mini(i);
				else if (aabb_.maxi(i) < p_(i))
					(*proximate_)(i) = aabb_.maxi(i);
				else
					(*proximate_)(i) = p_(i);		
			}
		}

		///@brief	AABB-AABBの衝突を判定する
		///@param	aabbA_	AABB_A
		///@param	aabbB_	AABB_B
		///@retval	true		衝突あり
		///@retval	false		衝突なし
		///@note	center-size型のAABBなら、大きなAABBと点に
		/// 			射影して衝突を判定した方が得かも。
		/// 			mini-maxi型のAABBならその限りではない？
		bool AabbAabbCol(
			const CAabb2&	aabbA_,
			const CAabb2&	aabbB_)
		{
			for (int i=0; i<2; i++)
			{
				if (aabbA_.mini(i) > aabbB_.maxi(i))
					return false;
				if (aabbA_.maxi(i) > aabbB_.mini(i))
					return false;
			}
			return true;
		}

		// ------------------------------------------------------------------
		// 3d
		// ------------------------------------------------------------------

		///@brief	回転体の位置と速度と角速度から、ある点の速度を求める
		///@param	world_		回転体のワールド変換行列
		///@param   v_			回転体の速度
		///@param	w_			回転体の角速度	
		///@param	p_			速度を求めたい点の回転体ローカル座標
		///@param   sweepVec_	[out]ワールドでの点の速度
		void RotSweep(
			const t_matrix4&	T_,		// 回転中心の位置、現在の姿勢
			const t_vector3&	r_,		// 回転中心からのローカル座標位置
			const t_vector3&	v_,		// 速度
			const t_vector3&	w_,		// 角速度
			t_vector3*			sweepVec_)	// 結果：スイープのベクトル
		{
			// ローカルでの角速度
			t_vector3 relW;
			MakeVectorCoordinateTransformed(&relW, T_, w_);

			// rの点の回転による速度
			t_vector3 v;
			MakeVectorCross(&v, r_, relW);

			// 回転による速度をワールドに変換
			(*sweepVec_)(0) = v(0)*T_(0, 0);
			(*sweepVec_)(1) = v(1)*T_(1, 0);
			(*sweepVec_)(2) = v(2)*T_(2, 0);
	
			// 並進による速度を加算
			*sweepVec_ += v_;
		}


		///@brief	点-点の距離を求める
		///@param	a_	点Aの座標
		///@param   b_	点Bの座標
		///@retval  点-点の距離
		float PtPtDistance(
			const t_vector3&	A_,
			const t_vector3&	B_)
		{
			t_vector3 ab = A_ - B_;
			return ab.Length();
		}

		///@brief	円-円の衝突を判定する
		///@param	centerA_	円Aの中心座標
		///@param	radiusA_	円Aの半径
		///@param   centerB_	円Bの中心座標
		///@param   radiusB_	円Bの半径
		///@retval  点-点の距離
		float SphereSphereCol(
			const t_vector3&	centerA_,
			float 				radiusA_,
			const t_vector3&	centerB_,
			float 				radiusB_)
		{
			t_vector3 ab(centerA_ - centerB_);
			float r = radiusA_ + radiusB_;

			// ２乗値で比較する方が早い
			return ab.LengthSquared() < r*r;
		}


		///@brief	点-線の距離を求める
		///@param	p_			点の座標
		///@param	lineP_		線上の任意の点
		///@param   lineDir_	正規化された線の方向
		///@retval  点-線の距離
		float LinePtDistance(
			const t_vector3&	p_,					
			const t_vector3&	lineP_,
			const t_vector3&	lineDir_)
		{
			// 方向ベクトルは正規化されているかチェック
			assert(lineDir_.Length() - 1.0 < 0.01f);

			t_vector3 pp = p_ - lineP_;
			t_vector3 cross = Cross(pp, lineDir_);
			float area = cross.Length();

			return fabs(area*lineDir_.ReciprocalLength());
		}

		///@brief	線分上にある点p_の最近接点を求める
		///@param	p_			点の座標
		///@param	segP_		線分の始点
		///@param   segQ_		線分の終点
		///@param   proximate_	[out]最近接点
		void ::rf::collision::SegPtProximate(				
			const t_vector3&	segP_,				
			const t_vector3&	segQ_,
			const t_vector3&	p_,					
			t_vector3*			proximate_)
		{
#if 0
			t_vector3 pp = p_ - segP_;
			t_vector3 pq(segQ_-segP_);
			t_vector3 dir;
			MakeVectorNormalized(&dir, pq);

			float t = Dot(pp, pq);
			*proximate_ = dir * t;
#endif
			
			if (Dot(t_vector3(segQ_-segP_), t_vector3(p_-segP_)) < 0)
			{
				*proximate_ = segP_;
			}	
			else if (Dot(t_vector3(segP_-segQ_), t_vector3(p_-segQ_)) < 0)
			{	
				*proximate_ = segQ_;
			}
			else
			{
				float dir_len = Length(t_vector3(segQ_-segP_));
				t_vector3 dir = t_vector3(segQ_-segP_) / dir_len;
				//MakeVectorNormalized(&dir, t_vector3(segQ_-segP_));
				//float t = Length(Cross(t_vector3(segQ_-segP_), t_vector3(p_-segP_)))
				//	/ Length(t_vector3(segQ_-segP_));
				float t = Dot(dir, t_vector3(p_-segP_));
				*proximate_ = dir * t + segP_;
			}
			return;
		}

		#if 0
			///@brief	線分上にある点p_の最近接点を求める
			///@param	p_			点の座標
			///@param	segP_		線分の始点
			///@param   segQ_		線分の終点
			///@param   proximate_	[out]最近接点
			void SegPtProximate(				
				const t_vector3&	segP_,				
				const t_vector3&	segQ_,
				const t_vector3&	p_,					
				t_vector3*			proximate_)
			{
				t_vector3 pqDir;
				t_vector3 pp = p_ - segP_;
				MakeVectorNormalized(t_vector3(segQ_ - segP_), &pqDir);
				float t = Dot(pp, pqDir);
				Clamp(&t, 0, 1);
				*proximate_ = t*pqDir;
			}
		#endif

		///@brief	点-線分の距離を求める
		///@param	p_			点の座標
		///@param	segP_		線分の始点
		///@param   segQ_		線分の終点
		///@retval	点-線分の距離
		float SegPtDistance(
			const t_vector3&	p_,					
			const t_vector3&	segP_,	
			const t_vector3&	segQ_)
		{
			#if 0
				t_vector3 ab = p_ - segP_;					
				t_vector3 ac = segQ_ - segP_;		
				t_vector3 bc = p_- segQ_;
				float e = Dot(ac, ab);
				if(e < 0)
					return ac.Length();
				float f = Dot(ab, ab);
					return bc.Length();
			#else
				t_vector3 proximate;
				SegPtProximate(segP_, segQ_, p_, &proximate);
				return Length(t_vector3(proximate - p_));
			#endif
		}

		///@brief	線-線の距離を求める
		///@param	segAP_		線Aの始点
		///@param	segADir_	線Aの終点
		///@param   segBP_		線Bの始点
		///@param   segBDir_	線Bの終点
		///@retval	線-線の距離
		float LineLineDistance(
			const t_vector3&	segAP_,		
			const t_vector3&	segADir_,	
			const t_vector3&	segBP_,		
			const t_vector3&	segBDir_)
		{
			t_vector3 pp = segBP_ - segAP_;
			t_vector3 cross = Cross(segADir_, segBDir_);
			return Dot(cross, pp)*ReciprocalLength(cross);
		}

		///@brief	線-線の最近接点を求める
		///@param	segAP_		線Aの始点
		///@param	segADir_	線Aの終点
		///@param   segBP_		線Bの始点
		///@param   segBDir_	線Bの終点
		///@param	proximateA_	[out]線A上の最近接点
		///@param	proximateB_	[out]線B上の最近接点
		///@note
		///
		/// 	最近接点を結んだ線分は両方の直線に垂直という条件
		///
		/// 		D1・V(s, t) = 0
		/// 		D2・V(s, t) = 0
		///
		void LineLineProximate(
			const t_vector3&	lineAP_,		
			const t_vector3&	lineADir_,		
			const t_vector3&	lineBP_,		
			const t_vector3&	lineBDir_,
			t_vector3*			proximateA_,	
			t_vector3*			proximateB_)
		{
			t_vector3 pp = lineAP_ - lineBP_;
			float a = Dot(lineADir_, lineADir_);
			float b = Dot(lineADir_, lineBDir_);
			float c = Dot(lineADir_, pp);
			float e = Dot(lineBDir_, lineBDir_);
			float f = Dot(lineBDir_, pp);
			float d = a*e-b*b;
			float s = (b*f-c*e)/d;
			float t = (a*f-b*c)/d;
			*proximateA_ = lineAP_ + s*lineADir_;
			*proximateB_ = lineBP_ + t*lineBDir_;
		}

		///@brief	線分-線分の最近接点
		///@param	segAP_		線Aの始点
		///@param	segADir_	線Aの終点
		///@param   segBP_		線Bの始点
		///@param   segBDir_	線Bの終点
		///@param	proximateA_	[out]線A上の最近接点
		///@param	proximateB_	[out]線B上の最近接点
		///@note
		///
		/// 	線分-線分の最近接点
		/// 	直線として最近接点を求め、それが線分の外側である場合は、
		/// 	線分の近側の端点と直線との最近接点を求める。
		/// 	こ最近接点がともに線分の外側にあるときは
		/// 	端点同士が最近接点
		///
		void ::rf::collision::SegSegProximate(
			const t_vector3&	segAP_,			
			const t_vector3&	segAQ_,				
			const t_vector3&	segBP_,				
			const t_vector3&	segBQ_,
			t_vector3*		proximateA_,
			t_vector3*		proximateB_)
		{
			t_vector3 dirA = segAQ_ - segAP_;
			t_vector3 dirB = segBQ_ - segBP_;
			t_vector3 pp = dirA - dirB;
			float a = Dot(dirA, dirA);
			float b = Dot(dirA, dirB);
			float c = Dot(dirA, pp);
			float e = Dot(dirB, dirB);
			float f = Dot(dirB, pp);
			float d = a*e-b*b;
			float s = (b*f-c*e)/d;
			float t = (a*f-b*c)/d;
	
			Clamp(&s, 0.0f, 1.0f);
			t = (b*s+f)/e;
			if(t < 0)
			{
				t = 0;
				s = -c/a;
				Clamp(&s, 0.0f, 1.0f);
			}
			if(t > 1.0f)
			{
				t = 1.0f;
				s = (b-c)/a;
				Clamp(&s, 0.0f, 1.0f);
			}
			*proximateA_ = s*dirA;
			*proximateB_ = t*dirB;
		}

		///@brief	平面-点の位置関係を判定する
		///@param	planeP_		平面上の任意の点
		///@param	planeN_		平面の法線
		///@param   p_			点の座標
		///@retval	true		表側
		///@retval	false		裏側
		bool PlanePtClassifyInside(
			const t_vector3&	planeP_,		
			const t_vector3&	planeN_,		
			const t_vector3&	p_)			
		{
			t_vector3 pp = p_ - planeP_;
			return Dot(pp, planeN_) < 0;
		}

		///@brief	平面-点の位置関係を判定する
		///@param	apex1_		平面上の任意の点A
		///@param	apex2_		平面上の任意の点B
		///@param	apex3_		平面上の任意の点C
		///@param   p_			点の座標
		///@retval	true		表側
		///@retval	false		裏側
		///@note	平面の裏表の定義は左手系に準じる
		bool PlanePtClassifyInside(
			const t_vector3&	apex1_,			
			const t_vector3&	apex2_,
			const t_vector3&	apex3_,
			const t_vector3&	p_)		
		{
			t_vector3 n;
			t_vector3 ab = apex2_ - apex1_;
			t_vector3 ac = apex3_ - apex1_;
			t_vector3 ap = p_ - apex1_;

			return Dot(ap, n) < 0;
		}

		///@brief	平面と点の符号付距離を求める
		///@param	plane_		平面
		///@param	p_			平面の法線
		///@param   depth_		[out]平面と点の符号付距離
		///@retval  平面と点の符号付距離、負が表側
		///@note	平面の裏表の定義は左手系に準じる
		float ::rf::collision::PlanePtDistance(
			const CPlane&			plane_,
			const math::t_vector3&	p_)
		{
			float d = Dot(plane_.normal(), p_);
			return d - plane_.d();
		}

		///@brief	平面と点の符号付距離を求める
		///@param	planeP_		平面上の任意の点
		///@param	planeN_		平面の法線
		///@param	p_			平面の法線
		///@param   depth_		[out]平面と点の符号付距離
		///@retval  平面と点の符号付距離、負が表側
		///@note	平面の裏表の定義は左手系に準じる
		///@note	本来はPの位置が分からなくても距離が求まるので少し損
		float ::rf::collision::PlanePtDistance(
			const math::t_vector3&	planeP_,			
			const math::t_vector3&	planeN_,
			const math::t_vector3&	p_)
		{
			math::t_vector3 normal;

			// 法線	
			MakeVectorNormalized(&normal, planeN_);
	
			// 法線方向の点の位置
			float p = Dot(p_, normal);

			// 法線方向の平面上の点の位置
			float d = Dot(planeP_, normal);

			return d - p;
		}

		///@brief	平面と点の符号付距離を求める
		///@param	apex1_		平面上の任意の点A
		///@param	apex2_		平面上の任意の点B
		///@param	apex3_		平面上の任意の点C
		///@param	p_			平面の法線
		///@param   depth_		[out]平面と点の符号付距離
		///@param   normal_		[out]平面の法線
		///@note	平面の裏表の定義は左手系に準じる
		float ::rf::collision::PlanePtDistance(
			const t_vector3&	apex1_,	
			const t_vector3&	apex2_,
			const t_vector3&	apex3_,
			const t_vector3&	p_,		
			float*				depth_,		
			math::t_vector3*	normal_)
		{
			t_vector3 penetrate_;
			t_vector3 ab = apex2_ - apex1_;
			t_vector3 ac = apex3_ - apex1_;
			t_vector3 ap = p_ - apex1_;
	
			// 法線
			MakeVectorCross(normal_, ab, ac);		
			normal_->Normalize();

			// 食い込みの投影
			*depth_ = Dot(ap, *normal_);

			return *depth_;
		}
		///@brief	平面-線分の衝突を判定する
		///@param	planeP_			平面上の任意の点
		///@param	planeN_			平面の法線
		///@param	segP_			線分の始点
		///@param	segQ_			線分の終点
		///@param   intersection_	[out]衝突点の座標、省略可
		///@retval	true			衝突あり
		///@retval	false			衝突なし
		///@note
		///
		/// 	法線の原点と目的の点を結んだベクトルと法線ベクトルの内積を求める。
		/// 	内積が正ならば表側、負ならば裏側なので、両端の点の符号が異なればいい
		///
		bool PlaneSegCol(
			const t_vector3&	planeP_,				
			const t_vector3&	planeN_,				
			const t_vector3&	segP_,					
			const t_vector3&	segQ_,					
			t_vector3*			intersection_)
		{
			// 相対位置に変換
			t_vector3 p1 = segP_ - planeP_;
			t_vector3 p2 = segQ_ - planeP_;
	
			float distanceS = Dot(p1, planeN_);
			float distanceE = Dot(p2, planeN_);

			if(distanceS * distanceE > 0)
				return false; 
	
			if(intersection_ == NULL)
				return true;

			*intersection_ = (distanceS / 
				(distanceS+distanceE)) * (p2 - p1);
			return true;
		}

		///@brief	平面と半直線の衝突を判定する
		///@param	planeP_			平面上の任意の点
		///@param	planeN_			平面の法線
		///@param	rayP_,			半直線の始点
		///@param	rayDir_			半直線の終点
		///@param   intersection_	[out]衝突点の座標、省略可
		///@retval	true			衝突あり
		///@retval	false			衝突なし
		// 上とほとんど同じ
		bool PlaneRayCol(
			const t_vector3&	planeP_,				
			const t_vector3&	planeN_,				
			const t_vector3&	rayP_,					
			const t_vector3&	rayVec_,				
			t_vector3*			intersection_)	
		{
			// 相対位置に変換
			t_vector3 p1 = rayP_ - planeP_;
			t_vector3 p2 = rayVec_ + p1;
	
			float distanceS = Dot(p1, planeN_);
			float distanceE = Dot(p2, planeN_);

			if(distanceS * distanceE > 0)
				return false; 
	
			if(intersection_ == NULL)
				return true;

			*intersection_ = (distanceS /
				(distanceS+distanceE)) * (p2 - p1);
			return true;
		}

		///@brief	片面三角-線の衝突位置を求める
		///@param	apex1_			三角形の頂点A
		///@param	apex2_			三角形の頂点B
		///@param	apex3_			三角形の頂点C
		///@param	lineP_			線上の任意の点
		///@param	lineDir_		線の方向
		///@param   intersection_	[out]衝突点の座標、省略可
		///@retval	true			衝突あり
		///@retval	false			衝突なし
		///@note
		///
		/// 	三角形の裏表の定義は左手系に準じる
		/// 	線分と平面の判定の後、start→交点・三角形上の二点の3つのベクトルの
		/// 	スカラ三重積(体積)がすべて同じ符号なら三角形内部
		///
		bool TriLineCol(
			const t_vector3&	apex1_,					
			const t_vector3&	apex2_,
			const t_vector3&	apex3_,
			const t_vector3&	lineP_,					
			const t_vector3&	lineDir_,
			t_vector3*			normal_,
			t_vector3*			intersection_)
		{
			t_vector3 vecPA = apex1_ - lineP_;
			t_vector3 vecPB = apex2_ - lineP_;
			t_vector3 vecPC = apex3_ - lineP_;
			t_vector3 vecPQ = lineDir_;

			// スカラ三重積
			// 交換法則によりQP×PCを使いまわす
			t_vector3 cross;
			MakeVectorCross(&cross, vecPQ, vecPC);

			float u = Dot(cross, vecPB);
			if(u < 0)
				return false;
			float v = - Dot(cross, vecPA);
			if(v < 0)
				return false;
			float w = ScalarTriple(vecPQ, vecPB, vecPA);
			if(w < 0)
				return false;
	
			// 線分が三角平面上じゃないか
			if(u<0.001f && v<0.001f && w<0.001f)
				return false;
	
			if (normal_ != NULL)
				MakeVectorNormalized(normal_, cross);

			if (intersection_ != NULL)
				*intersection_ = (u*vecPA+v*vecPB+w*vecPC)/(u+v+w)+lineP_;

			return true;
		}

		///@brief	片面三角-線分の衝突位置を求める
		///@param	apex1_			三角形の頂点A
		///@param	apex2_			三角形の頂点B
		///@param	apex3_			三角形の頂点C
		///@param	segP_			線分の始点
		///@param	segQ_			線分の終点
		///@param   distance_		[out]衝突点の座標、省略可
		///@param   intersection_	[out]衝突点の座標、省略可
		///@retval	true			衝突あり
		///@retval	false			衝突なし
		///@note
		///
		/// 	基本的には三角形-線と同じ
		/// 	三角形状の点がuA+vB+wCで表されるとき
		/// 		u+v+w=1なら平面上の点、最適化のためu=1-v-wで方程式を解くと
		/// 		v>0,w>0,v+w<1
		/// 	が最終的な条件
		/// 	基本的には三角形と線との判定と同じになるが、
		/// 	その計算途中に得られる各値を先に見ていくことで、
		/// 	衝突していない場合を早期に発見できる
		///
		bool TriSegCol(
			const t_vector3&	apex1_,
			const t_vector3&	apex2_,
			const t_vector3&	apex3_,
			const t_vector3&	segP_,
			const t_vector3&	segDir_,
			math::t_vector3*	normal_,
			float*				distance_,
			math::t_vector3*	intersection_)	
		{
			t_vector3 vecAB = apex2_ - apex1_;
			t_vector3 vecAC = apex3_ - apex1_;

			// 平面交差判定
			t_vector3 normal; 
			MakeVectorCross(&normal, vecAB, vecAC);

			// QP×nによる交差の前判定
			// QPの法線方向成分d>0なら方向違い
			// dによる正規化はしないで基準値として扱う
			float d = Dot(segDir_, normal);
			if(d > 0)
				return false; 

			// APとQPを法線方向に投影したパラメータ距離の符号を比較
			t_vector3 vecAP(segP_ - apex1_);
			float t = Dot(vecAP, normal);

			// すでに貫通している
			if(t < 0)
				return false;

			// |AP・n| < |d|なら線が届かない
			if(t > -d)
				return false;

			// スカラ三重積による交差判定
			t_vector3 e = Cross(segDir_, vecAP);
		
			// v = (C-A)・e/dにより、
			// vはdを掛けて正になる予定なので負である必要がある
			float v = Dot(vecAC, e);
			if(0 < v || v < d)
				return false;
			float w = - Dot(vecAB, e);
			if(0 < w || v+w < d)
				return false;
	
			// 線が三角形上
			if(v < 0.01f && w < 0.01f)
				return false;

			if (distance_ != NULL)
				*distance_ = t / -d;

			if (normal_ != NULL)
				MakeVectorNormalized(normal_, normal); 		

			// v /= d;
			// w /= d;
			// u /= 1 - v - w;

			return true;
		}


		///@brief	両面三角-線分の衝突位置を求める
		///@param	apex1_			三角形の頂点A
		///@param	apex2_			三角形の頂点B
		///@param	apex3_			三角形の頂点C
		///@param	segP_			線分の始点
		///@param	segQ_			線分の終点
		///@param	normal_			[out]三角形の法線、省略不可
		///@param   distance_		[out]衝突点の座標、省略可
		///@param   intersection_	[out]衝突点の座標、省略可
		///@retval	true			衝突あり
		///@retval	false			衝突なし
		///@note
		///
		/// 	基本的には三角形-線と同じ
		/// 	三角形状の点がuA+vB+wCで表されるとき
		/// 		u+v+w=1なら平面上の点、最適化のためu=1-v-wで方程式を解くと
		/// 		v>0,w>0,v+w<1
		/// 	が最終的な条件
		/// 	基本的には三角形と線との判定と同じになるが、
		/// 	その計算途中に得られる各値を先に見ていくことで、
		/// 	衝突していない場合を早期に発見できる
		///
		/// 	両面の方が若干処理が重い
		///
		bool TriSegColBothFaces(
			const t_vector3&	apex1_,					
			const t_vector3&	apex2_,
			const t_vector3&	apex3_,
			const t_vector3&	segP_,					
			const t_vector3&	segQ_,					
			t_vector3*			normal_,
			float*				distance_,
			t_vector3*			intersection_)
		{
			// 各種ベクトル
			t_vector3 vecAB = apex2_ - apex1_;
			t_vector3 vecAC = apex3_ - apex1_;
			t_vector3 vecPQ = segQ_ - segP_;

			// 法線
			t_vector3 normal; 
			MakeVectorCross(&normal, vecAB, vecAC);

			float d = Dot(vecPQ, normal);
	
			// 線分が三角平面に平行な場合を排除
			// 片面の場合は符号判定で早期終了可
			if(fabs(d) < 0.01f)
				return false;

			// APとQPの法線方向成分の符号を比較
			// 同じ場合は交差方向が正しいが
			// |AP・n| < |d|なら線が届かない
			t_vector3 vecAP(segP_ - apex1_);
			float t = Dot(vecAP, normal);
			if ((fabs(t) > fabs(d) + RF_EPSILON)
			||  (t*d > 0))
				return false;

			// 事前に解いた式を用いてt,v,wを求める。
			t_vector3 e; // 計算用
			MakeVectorCross(&e, vecPQ, vecAP); 

			// v = (C-A)・e/d
			float v = Dot(vecAC, e);
			if(v*d < 0 || fabs(v) > fabs(d)+RF_EPSILON)
				return false;

			// v = (B-A)・e/d
			float w = - Dot(vecAB, e);
			if(w*d < 0 || fabs(v+w) > fabs(d)+RF_EPSILON)
				return false;

			// v /= d;
			// w /= d;
			// u /= 1 - v - w; 

			if(distance_ != NULL)
				*distance_ = t / -d;

			if(intersection_ != NULL)
				*intersection_ = t*vecPQ + segP_;

			if(normal_ != NULL)
				MakeVectorNormalized(normal_, normal);

			return true;
		}

		///@brief	三角ポリゴンと線分の最近接点
		///@param	apexA1_			三角形の頂点1
		///@param	apexA2_			三角形の頂点2
		///@param	apexA3_			三角形の頂点3
		///@param	segP_			線分の始点
		///@param	segQ_			線分の終点
		///@param	proximateL_		[out]線上の最近接点
		///@param	proximateP_		[out]三角形上の最近接点
		///@note
		///
		/// 	三角形の裏表の定義は左手系に準じる
		/// 	三角ポリゴンと線分の最近接点
		/// 	辺対線分（3種類）と、三角形対線の端点（2種類）
		/// 	のうち、もっとも距離の短かったもの。
		/// 	辺対線分の最近接点を全部の辺について求め、最小のものを求める
		/// 	辺と辺の距離を求めて、最小についてだけ最近接点を求めたほうが早いか？
		///
		void TriSegProximate(
			const t_vector3&	apex1_,			
			const t_vector3&	apex2_,
			const t_vector3&	apex3_,
			const t_vector3&	segP_,			
			const t_vector3&	segQ_,			
			t_vector3*			proximateL_,		
			t_vector3*			proximateP_)		
		{
			t_vector3 PQ = segQ_ - segP_;
			t_vector3 proximateL;
			t_vector3 proximateP; 
			float mini;
		
			// 辺１
			SegSegProximate(segP_, segQ_, apex1_, apex2_, proximateL_, proximateP_);
			t_vector3 proximate = *proximateP_ - *proximateL_;
			mini = Length(proximate);
	
			// 辺２
			SegSegProximate(segP_, segQ_, apex2_, apex3_, &proximateL, &proximateP);
			proximate = proximateP - proximateL;
			float len = proximate.Length();
			if(mini > len)
			{
				mini = len;
				*proximateL_ = proximateL;
				*proximateP_ = proximateP;
			}

			// 辺３
			SegSegProximate(segP_, segQ_, apex3_, apex1_, &proximateL, &proximateP);
			proximate = proximateP - proximateL;
			len = proximate.Length();
			if(mini > len)
			{
				mini = len;
				*proximateL_ = proximateL;
				*proximateP_ = proximateP;
			}
		}
		// 三角ポリゴンと三角ポリゴン画衝突していないことを判定
		// 衝突していなければtrue
		///@brief	両面三角-両面三角の衝突を判定する
		///@param	apexA1_			三角形Aの頂点1
		///@param	apexA2_			三角形Aの頂点2
		///@param	apexA3_			三角形Aの頂点3
		///@param	apexB1_			三角形Bの頂点1
		///@param	apexB2_			三角形Bの頂点2
		///@param	apexB3_			三角形Bの頂点3
		///@retval	true			衝突しているかもしれない
		///@retval	false			確実に衝突していない			
		bool TriTriColPosibility(
			const t_vector3&	apexA1_,				// ポリゴンの頂点
			const t_vector3&	apexB1_,
			const t_vector3&	apexC1_,
			const t_vector3&	apexA2_,				// ポリゴンの頂点
			const t_vector3&	apexB2_,
			const t_vector3&	apexC2_)
		{
			t_vector3 n;
			float dot;

			// １側
			t_vector3 tmp1 = apexB1_ - apexA1_;
			t_vector3 tmp2 = apexC1_ - apexA1_;
			t_vector3 tmp3 = apexA2_ - apexA1_;
			t_vector3 tmp;
			MakeVectorCross(&n, tmp1, tmp2);
			dot = Dot(n, tmp3);
			if(dot>=0)
			{
				tmp = apexB2_ - apexA1_;
				if(Dot(n, tmp)>=0)
				{
					tmp = apexC2_ - apexA1_;
					if(Dot(n, tmp)>=0)
						return true;
				}
			}
			else// dot1<0
			{
				tmp = apexB2_ - apexA1_;
				if(Dot(n, tmp)>=0)
				{
					tmp = apexC2_ - apexA1_;
					if(Dot(n, tmp)>=0)
						return true;
				}
			}

			// 2側
			tmp1 = apexB2_ - apexA2_;
			tmp2 = apexC2_ - apexA2_;
			tmp3 = apexA1_ - apexA2_;
			MakeVectorCross(&n, tmp1, tmp2);
			dot = Dot(n, tmp3);
			if(dot>=0)
			{
		
				tmp = apexB1_ - apexA2_;
				if(Dot(n, tmp)>=0)
				{
					tmp = apexC1_ - apexA2_;
					if(Dot(n, tmp)>=0)
						return true;
				}
			}
			else
			{
				// dot1<0

				tmp = apexB1_ - apexA2_;
				if(Dot(n, tmp)<0)
				{
					tmp = apexC1_ - apexA2_;
					if(Dot(n, tmp)<0)
						return true;
				}
			}
		
			return false;

		}


		#if 0

		///@brief	片面三角-片面三角の衝突点と衝突方向を求める
		/// 			三角形の面-頂の衝突の場合面側は。。。
		/// 			面と頂点の衝突を両面ともとる。。。
		///@param	apexA1_			三角形Aの頂点1
		///@param	apexA2_			三角形Aの頂点2
		///@param	apexA3_			三角形Aの頂点3
		///@param	apexB1_			三角形Bの頂点1
		///@param	apexB2_			三角形Bの頂点2
		///@param	apexB3_			三角形Bの頂点3
		///@param   intersectionA_	[out]三角形A側の衝突点座標、省略可
		///@param   intersectionB_	[out]三角形B側の衝突点座標、省略可
		///@param   penetrateA_		[out]三角形AのBに対する進入量、省略可
		///@note
		///
		/// 	この関数は最適化が可能だと思われる
		/// 	三角ポリゴンと三角ポリゴン
		/// 	内部で辺-辺か、頂点-面の衝突か判定し、
		/// 	辺-辺の場合はそれぞれの最近接点、
		/// 	頂点-面の場合は頂点を面に投影した点を
		/// 	hitPos1_、hitPos2_に書き込む。
		///
		e_collide_bits TriTriCol(
			const t_vector3&	apexA1_,				
			const t_vector3&	apexA2_,
			const t_vector3&	apexA3_,
			const t_vector3&	apexB1_,				
			const t_vector3&	apexB2_,
			const t_vector3&	apexB3_,
			math::t_vector3*	intersectionA_ = NULL,	
			math::t_vector3*	intersectionB_ = NULL,
			math::t_vector3*	penetrate_ = NULL);	
		{
			enum HIT_FLAG			// for 三角形対三角形
			{
				HIT_AB1 = 1,
				HIT_BC1 = 1<<1,
				HIT_CA1 = 1<<2,
				HIT_AB2 = 1<<3,
				HIT_BC2 = 1<<4,
				HIT_CA2 = 1<<5,
			};

			// 最初に、どう衝突するかを調べる
			int hitCount1 = 0;
			int hitFlag1 = 0;
			int hitCount2 = 0;
			int hitFlag2 = 0;

			// １の面に対する２の辺
			if(TriSegColBothFaces(apexA1_, apexA2_, apexA3_, apexB1_, apexB2_, NULL)==true)
			{
				hitFlag1 |= HIT_AB2;
				hitCount1++;
			}
			if(TriSegColBothFaces(apexA1_, apexA2_, apexA3_, apexB2_, apexB3_, NULL)==true)
			{
				hitFlag1 |= HIT_BC2;
				hitCount1++;
			}
			if(hitCount1 < 2)
			if(TriSegColBothFaces(apexA1_, apexA2_, apexA3_, apexB3_, apexB1_, NULL)==true)
			{
				hitFlag1 |= HIT_CA2;
				hitCount1++;
			}

			// ２の面に対する１の辺
			if(TriSegColBothFaces(apexB1_, apexB2_, apexB3_, apexA1_, apexA2_, NULL)==true)
			{
				hitFlag2 |= HIT_AB1;
				hitCount2++;
			}
			if(TriSegColBothFaces(apexB1_, apexB2_, apexB3_, apexA2_, apexA3_, NULL)==true)
			{
				hitFlag2 |= HIT_BC1;
				hitCount2++;
			}
			if(hitCount2 < 2)
			if(TriSegColBothFaces(apexB1_, apexB2_, apexB3_, apexA3_, apexA1_, NULL)==true)
			{
				hitFlag2 |= HIT_CA1;
				hitCount2++;
			}
	
			// ヒットカウント2は面対頂点
			if(hitCount1 == 2 && hitCount2 == 0)
			{
				float depth;
				t_vector3 n;
				t_vector3 tmp1 = apexA2_-apexA1_;
				t_vector3 tmp2 = apexA3_-apexA1_;
				Cross(tmp1, tmp2, &n);
				n.Normalize();
				switch(hitFlag1)
				{
				case HIT_AB2 | HIT_BC2:
					tmp1 = apexB2_ - apexA1_;
					depth = Dot(tmp1, n);
					if(depth > 0)
						return RF_COLLIDE_NONE;
					if (hitPos1_ != NULL)
						*hitPos1_ = apexB2_
					if (hitPos2_ != NULL)
						*hitPos2_ = apexB2_;

					break;

				case HIT_CA2 | HIT_BC2:
					tmp1 = apexB3_ - apexA1_;
					depth = Dot(tmp1, n);
					if(depth > 0)
						return RF_COLLIDE_NONE;
					if (hitPos1_ != NULL)
						*hitPos1_ = apexB3_
					if (hitPos2_ != NULL)
						*hitPos2_ = apexB3_;
					break;

					#if 1
						// case HIT_AB2 | HIT_CA2:
						default:
							tmp1 = apexB1_ - apexA1_;
							depth = Dot(tmp1, n);
							if(depth > 0)
								return RF_COLLIDE_NONE;
							if (hitPos1_ != NULL)
								*hitPos1_ = apexB1_
							if (hitPos2_ != NULL)
								*hitPos2_ = apexB1_;
							break;

					#else
						case HIT_AB2 | HIT_CA2:
							tmp1 = apexB1_ - apexA1_;
							depth = Dot(tmp1, n);
							if(depth > 0)
								return RF_COLLIDE_NONE;
							if (hitPos1_ != NULL)
								*hitPos1_ = apexB1_
							if (hitPos2_ != NULL)
								*hitPos2_ = apexB1_;
							break;

						default:
							depth=0;
							RFASSERT(false, NULL);
					#endif
				}
				*penetrate_ = n * depth;
				return RF_COLLIDE_VERTEX;
			}

			// ヒットカウント2は面対頂点
			if(hitCount2 == 2 && hitCount1 == 0)
			{
				float depth = 0;// 警告対策に０で初期化
				t_vector3 n;
				t_vector3 tmp1 = apexB2_-apexA2_;
				t_vector3 tmp2 = apexC2_-apexA2_;
				Cross(&n, &tmp1, &tmp2);	// 法線
				n = n /D3DXVec3Length(&n);
				switch(hitFlag2)
				{
				case HIT_AB1 | HIT_BC1:
					tmp1 = apexA2_ - apexB1_;
					depth = Dot(&tmp1, &n);
					if(depth > 0)
						return RF_COLLIDE_NONE;
					if (hitPos1_ != NULL)
						*hitPos1_ = apexA2_;
					if (hitPos2_ != NULL)
						*hitPos2_ = apexA2_;
					break;

				case HIT_CA1 | HIT_BC1:
					tmp1 = apexA3_ - apexB1_;
					depth = Dot(&tmp1, &n);
					if(depth > 0)
						return RF_COLLIDE_NONE;
					if (hitPos1_ != NULL)
						*hitPos1_ = apexA3_;
					if (hitPos2_ != NULL)
						*hitPos2_ = apexA3_;
					break;

				case HIT_AB1 | HIT_CA1:
					tmp1 = apexA1_ - apexB1_;
					depth = Dot(&tmp1, &n);
					if(depth > 0)
						return RF_COLLIDE_NONE;
					if (hitPos1_ != NULL)
						*hitPos1_ = apexA1_;
					if (hitPos2_ != NULL)
						*hitPos2_ = apexA1_;
					break;

				default:
					RFASSERT(false, NULL);
				}
				*penetrate_ = n * -depth;

				return RF_COLLIDE_VERTEX;
			}

			// 辺の衝突
			// 頂点-辺も含める
			if(hitCount1 > 0 && hitCount2 > 0)
			{
				int flag = hitFlag2|hitFlag1;
		
				if((flag&(HIT_AB1|HIT_AB2)) == (HIT_AB1|HIT_AB2))
					LineLineProximate(apexA1_, apexA2_ - apexA1_, apexB1_, apexB2_ - apexB1_, hitPos1_, hitPos2_);
				else if((flag&(HIT_AB1 | HIT_BC2)) == (HIT_AB1 | HIT_BC2))
					LineLineProximate(apexA1_, apexA2_ - apexA1_, apexB2_, apexB3_ - apexB2_, hitPos1_, hitPos2_);
				else if((flag&(HIT_AB1 | HIT_CA2)) == (HIT_AB1 | HIT_CA2))
					LineLineProximate(apexA1_, apexA2_ - apexA1_, apexB3_, apexB1_ - apexB3_, hitPos1_, hitPos2_);

				else if((flag&(HIT_BC1 | HIT_AB2)) == (HIT_BC1 | HIT_AB2))
					LineLineProximate(apexA3_, apexA2_ - apexA3_, apexB1_, apexB2_ - apexB1_, hitPos1_, hitPos2_);
				else if((flag&(HIT_BC1 | HIT_BC2)) == (HIT_BC1 | HIT_BC2))
					LineLineProximate(apexA3_, apexA2_ - apexA3_, apexB2_, apexB3_ - apexB2_, hitPos1_, hitPos2_);
				else if((flag&(HIT_BC1 | HIT_CA2)) == (HIT_BC1 | HIT_CA2))
					LineLineProximate(apexA3_, apexA2_ - apexA3_, apexB1_, apexB3_ - apexB1_, hitPos1_, hitPos2_);

				else if((flag&(HIT_CA1 | HIT_AB2)) == (HIT_CA1 | HIT_AB2))
					LineLineProximate(apexA1_, apexA3_ - apexA1_, apexB1_, apexB2_ - apexB1_, hitPos1_, hitPos2_);
				else if((flag&(HIT_CA1 | HIT_BC2)) == (HIT_CA1 | HIT_BC2))
					LineLineProximate(apexA1_, apexA3_ - apexA1_, apexB3_, apexB2_ - apexB3_, hitPos1_, hitPos2_);
				else if((flag&(HIT_CA1 | HIT_CA2)) == (HIT_CA1 | HIT_CA2))
					LineLineProximate(apexA1_, apexA3_ - apexA1_, apexB1_, apexB3_ - apexB1_, hitPos1_, hitPos2_);
				else
					RFASSERT(false, NULL);	
		
				// 食い込み算出
				*penetrate_ =  *hitPos2_ - *hitPos1_;
				return RF_COLLIDE_EDGE;
			}

			// 衝突なし
			return RF_COLLIDE_NONE;
		}


		/**********************************************************/
		// !!!この関数は最適化が可能だと思われる
		// 三角ポリゴンと三角ポリゴン
		// 内部で辺-辺か、頂点-面の衝突か判定し、
		// 辺-辺の場合はそれぞれの最近接点、
		// 頂点-面の場合は頂点を面に投影した点を
		// hitPos1_、hitPos2_に書き込む。
		e_collide_bits TriTriColBothFace(
			const t_vector3&	apexA1_,				// ポリゴンの頂点
			const t_vector3&	apexA2_,
			const t_vector3&	apexA3_,
			const t_vector3&	apexB1_,				// ポリゴンの頂点
			const t_vector3&	apexB2_,
			const t_vector3&	apexB3_,
			t_vector3*			hitPos1_,				// 衝突位置
			t_vector3*			hitPos2_,
			t_vector3*			penetrate_)				// めり込みの方向// 法線
		{
			// for 三角形対三角形
			enum HIT_FLAG
			{
				HIT_AB1 = 1,
				HIT_BC1 = 1<<1,
				HIT_CA1 = 1<<2,
				HIT_AB2 = 1<<3,
				HIT_BC2 = 1<<4,
				HIT_CA2 = 1<<5,
			};

			// 最初に、どう衝突するかを調べる
			int hitCount1 = 0;
			int hitFlag1 = 0;
			int hitCount2 = 0;
			int hitFlag2 = 0;

			// １の面に対する２の辺
			if(TriSegColBothFaces(apexA1_, apexA2_, apexA3_, apexB1_, apexB2_, NULL)==true)
			{
				hitFlag1 |= HIT_AB2;
				hitCount1++;
			}
			if(TriSegColBothFaces(apexA1_, apexA2_, apexA3_, apexB2_, apexB3_, NULL)==true)
			{
				hitFlag1 |= HIT_BC2;
				hitCount1++;
			}
			if(hitCount1 < 2)
			if(TriSegColBothFaces(apexA1_, apexA2_, apexA3_, apexB3_, apexB1_, NULL)==true)
			{
				hitFlag1 |= HIT_CA2;
				hitCount1++;
			}

			// ２の面に対する１の辺
			if(TriSegColBothFaces(apexB1_, apexB2_, apexB3_, apexA1_, apexA2_, NULL)==true)
			{
				hitFlag2 |= HIT_AB1;
				hitCount2++;
			}
			if(TriSegColBothFaces(apexB1_, apexB2_, apexB3_, apexA2_, apexA3_, NULL)==true)
			{
				hitFlag2 |= HIT_BC1;
				hitCount2++;
			}
			if(hitCount2 < 2)
			if(TriSegColBothFaces(apexB1_, apexB2_, apexB3_, apexA3_, apexA1_, NULL)==true)
			{
				hitFlag2 |= HIT_CA1;
				hitCount2++;
			}
	
			// ヒットカウント2は面対頂点が確定
			// !!!ここ以降に来るときは頂点の同時衝突が起きていることを疑ったほうがいい
			// !!!頂点スウィープをすべての頂点について行った後、
			// 一番食い込んでいた点を衝突点とするような処理をすべき
			if(hitCount1 == 2 && hitCount2 == 0)
			{
				float depth = 0;// 警告対策に０で初期化
				t_vector3 n;
				t_vector3 tmp = apexA2_-apexA1_;  // 警告対策で初期化することにしたので、おかしくなったらここのせいだと思う
				t_vector3 tmp2 = apexA3_-apexA1_; // 警告対策
				Cross(&n, &tmp, &tmp2);	// 法線
				n = n /D3DXVec3Length(&n);
				switch(hitFlag1)
				{
				case HIT_AB2 | HIT_BC2:
					tmp = apexB2_ - apexA1_;
					depth = Dot(&tmp, &n);
					if(depth > 0)
					{
						tmp = apexB3_ - apexA1_;
						depth = Dot(&tmp, &n);
						*hitPos1_ = *hitPos2_ = apexB3_;
					}
					else
					{
						*hitPos1_ = *hitPos2_ = apexB2_;
					}
					break;
				case HIT_CA2 | HIT_BC2:
					tmp = apexB3_ - apexA1_;
					depth = Dot(&tmp, &n);
					if(depth > 0)
					{
						tmp = apexB1_ - apexA1_;
						depth = Dot(&tmp, &n);
						*hitPos1_ = *hitPos2_ = apexB1_;
					}
					else
					{
						*hitPos1_ = *hitPos2_ = apexB3_;
					}
					break;
				case HIT_AB2 | HIT_CA2:
					tmp = apexB1_ - apexA1_;
					depth = Dot(&tmp, &n);
					if(depth > 0)
					{
						tmp = apexB2_ - apexA1_;
						depth = Dot(&tmp, &n);
						*hitPos1_ = *hitPos2_ = apexB2_;
					}
					else
					{
						*hitPos1_ = *hitPos2_ = apexB1_;
					}
					break;
				default:
					RFASSERT(false, NULL);
				}
				*penetrate_ = n * depth;
				return HRK_COL_VERTEX;
			}

			// ヒットカウント2は面対頂点が確定
			if(hitCount2 == 2 && hitCount1 == 0)
			{
				float depth = 0;// 警告対策に０で初期化
				t_vector3 n;
				t_vector3 tmp = apexB2_-apexB1_;  // 警告対策で初期化することにしたので、おかしくなったらここのせいだと思う
				t_vector3 tmp2 = apexB3_-apexB1_; // 警告対策
				Cross(&n, &tmp, &tmp2);	// 法線
				n = n /D3DXVec3Length(&n);
				switch(hitFlag2)
				{
				case HIT_AB1 | HIT_BC1:
					tmp = apexA2_ - apexB1_;
					depth = Dot(&tmp, &n);
					if(depth > 0)
					{
						tmp = apexA3_ - apexB1_;
						depth = Dot(&tmp, &n);
						*hitPos1_ = *hitPos2_ = apexA3_;
					}
					else
					{
						*hitPos1_ = *hitPos2_ = apexA2_;
					}
					break;
				case HIT_CA1 | HIT_BC1:
					tmp = apexA3_ - apexB1_;
					depth = Dot(&tmp, &n);
					if(depth > 0)
					{
						tmp = apexA1_ - apexB1_;
						depth = Dot(&tmp, &n);
						*hitPos1_ = *hitPos2_ = apexA1_;
					}
					else
					{
						*hitPos1_ = *hitPos2_ = apexA3_;
					}
					break;
				case HIT_AB1 | HIT_CA1:
					tmp = apexA1_ - apexB1_;
					depth = Dot(&tmp, &n);
					if(depth > 0)
					{
						tmp = apexA2_ - apexB1_;
						depth = Dot(&tmp, &n);
						*hitPos1_ = *hitPos2_ = apexA2_;
					}
					else
					{
						*hitPos1_ = *hitPos2_ = apexA1_;
					}
					break;
				default:
					RFASSERT(false, NULL);
				}
				*penetrate_ = n * -depth;

				return HRK_COL_VERTEX;
			}

			// 辺の衝突
			// 頂点-辺も含める
			if(hitCount1 > 0 && hitCount2 > 0)
			{
				int flag = hitFlag2|hitFlag1;
		
				if((flag&(HIT_AB1|HIT_AB2)) == (HIT_AB1|HIT_AB2))
					LineLineProximate(apexA1_, apexA2_ - apexA1_, apexB1_, apexB2_ - apexB1_, hitPos1_, hitPos2_);
				else if((flag&(HIT_AB1 | HIT_BC2)) == (HIT_AB1 | HIT_BC2))
					LineLineProximate(apexA1_, apexA2_ - apexA1_, apexB2_, apexB3_ - apexB2_, hitPos1_, hitPos2_);
				else if((flag&(HIT_AB1 | HIT_CA2)) == (HIT_AB1 | HIT_CA2))
					LineLineProximate(apexA1_, apexA2_ - apexA1_, apexB3_, apexB1_ - apexB3_, hitPos1_, hitPos2_);

				else if((flag&(HIT_BC1 | HIT_AB2)) == (HIT_BC1 | HIT_AB2))
					LineLineProximate(apexA3_, apexA2_ - apexA3_, apexB1_, apexB2_ - apexB1_, hitPos1_, hitPos2_);
				else if((flag&(HIT_BC1 | HIT_BC2)) == (HIT_BC1 | HIT_BC2))
					LineLineProximate(apexA3_, apexA2_ - apexA3_, apexB2_, apexB3_ - apexB2_, hitPos1_, hitPos2_);
				else if((flag&(HIT_BC1 | HIT_CA2)) == (HIT_BC1 | HIT_CA2))
					LineLineProximate(apexA3_, apexA2_ - apexA3_, apexB1_, apexB3_ - apexB1_, hitPos1_, hitPos2_);

				else if((flag&(HIT_CA1 | HIT_AB2)) == (HIT_CA1 | HIT_AB2))
					LineLineProximate(apexA1_, apexA3_ - apexA1_, apexB1_, apexB2_ - apexB1_, hitPos1_, hitPos2_);
				else if((flag&(HIT_CA1 | HIT_BC2)) == (HIT_CA1 | HIT_BC2))
					LineLineProximate(apexA1_, apexA3_ - apexA1_, apexB3_, apexB2_ - apexB3_, hitPos1_, hitPos2_);
				else if((flag&(HIT_CA1 | HIT_CA2)) == (HIT_CA1 | HIT_CA2))
					LineLineProximate(apexA1_, apexA3_ - apexA1_, apexB1_, apexB3_ - apexB1_, hitPos1_, hitPos2_);
				else
					RFASSERT(false, NULL);	
		
				// 食い込み算出
				*penetrate_ =  *hitPos2_ - *hitPos1_;
				return HRK_COL_LINE;
			}

			// 衝突なし
			return false;
		}


		#endif

		///@brief	片面四角-線分の衝突位置を求める
		///@param	apex1_			四角形の頂点A
		///@param	apex2_			四角形の頂点B
		///@param	apex3_			四角形の頂点C
		///@param	apexD_			四角形の頂点D
		///@param	segP_			線分の始点
		///@param	segQ_			線分の終点
		///@param   intersection_	[out]衝突点の座標、省略可
		///@retval	true			衝突あり
		///@retval	false			衝突なし
		///@note	
		/// 	
		/// 	四角形の裏表の定義は左手系に準じる
		/// 	対角線で切って三角形の判定すると、対角線のどちら側にあるかで分岐
		///
		bool SquareSegCol(
			const t_vector3&	apex1_,			
			const t_vector3&	apex2_,
			const t_vector3&	apex3_,
			const t_vector3&	apexD_,
			const t_vector3&	segP_,			
			const t_vector3&	segQ_,
			t_vector3*			intersection_)
		{
			t_vector3 vecPA = apex1_ - segP_;
			t_vector3 vecPB = apex2_ - segP_;
			t_vector3 vecPC = apex3_ - segP_;
			t_vector3 vecPD = apexD_ - segP_;
			t_vector3 vecPQ = segQ_ - segP_;

			// スカラ三重積の交換法則よりQP×P1を使いまわす
			t_vector3 cross;
			MakeVectorCross(&cross, vecPQ, vecPC);

			float u = Dot(cross, vecPA);
			if(u < 0)
			{
				// ACB

				float v = Dot(cross, vecPB);
				if(v < 0)
					return false;
				float w = ScalarTriple(vecPQ, vecPB, vecPA);
				if(w < 0)
					return false;

				// 衝突位置は体積比による重心位置になる
				if(intersection_ != NULL)
				{
					u = -u;
					*intersection_ = (u*vecPB+v*vecPA+w*vecPC)
						/(u+v+w)+segP_;
				}
			}
			else
			{
				// ACD (u>0)側の三角形

				float v = -Dot(cross, vecPD);
				if(v < 0)
					return false;
				float w = ScalarTriple(vecPQ, vecPA, vecPD);
				if(w < 0)
					return false;

				// 衝突位置は体積比による重心位置になる
				if(intersection_ != NULL)
				{
					*intersection_ = (u*vecPD+v*vecPA+w*vecPC)
						/(u+v+w)+segP_;
				}
			}

			return true; 
		}

		#if 0
			///@brief	AABB-点の衝突
			///@param	aabb_	AABB
			///@param	pt_		点の座標
			///@retval	true	衝突あり
			///@retval	false	衝突なし
			eSTATUS AABBPtCol(						
				const t_vector3&	centerA_,
				const t_vector3&	halfSizeA_,
				const t_vector3&	point_)
			{
				t_vector3 offset;
				D3DXVec3Subtract(&offset, &point_, &centerA_);

				if (-halfSizeA_.x < offset.x && offset.x < halfSizeA_.x
				&&  -halfSizeA_.y < offset.y && offset.y < halfSizeA_.y
				&&  -halfSizeA_.z < offset.z && offset.z < halfSizeA_.z)
				{
					return true;
				}
				return false;
			}
		#else
			///@brief	AABB-点の衝突
			///@param	aabb_	AABB
			///@param	pt_		点の座標
			///@retval	true	衝突あり
			///@retval	false	衝突なし
			bool AabbPtCol(
				const CAabb3&	aabb_,
				const t_vector3&			pt_)
			{
				bool retval = (pt_(0) <= aabb_.mini(0) && pt_(0) <= aabb_.mini(0))
					&& (pt_(1) <= aabb_.mini(1) && pt_(1) <= aabb_.mini(1))
					&& (pt_(2) <= aabb_.mini(2) && pt_(2) <= aabb_.mini(2));
				return retval;
			}
		#endif


		///@brief	AABB-点最近接点を取得する
		///@param	aabb_			AABB
		///@param	p_				点の座標
		///@param	proximate_		[out]最近接点
		void rf::collision::AabbPtProximate(
			const CAabb3&		aabb_,	
			const t_vector3&	p_,
			t_vector3*			proximate_)
		{
			for (int i=0; i<3; i++)
			{
				if (p_(i) < aabb_.mini(i))
					(*proximate_)(i) = aabb_.mini(i);
				else if (aabb_.maxi(i) < p_(i))
					(*proximate_)(i) = aabb_.maxi(i);
				else
					(*proximate_)(i) = p_(i);		
			}
		}


		///@brief	AABB-球の衝突を判定する
		///@param	aabb_		AABB
		///@param	p_			点の座標
		///@param	radius_		球の半径
		bool AabbSphereCol(
			const CAabb3&		aabb_,	
			const t_vector3&	p_,
			float				radius_)
		{
			t_vector3 proximate;
			AabbPtProximate(aabb_, p_, &proximate);
			t_vector3 distance(p_ - proximate);
			return distance.LengthSquared() < radius_*radius_;
		}

		///@brief	AABB-光線の衝突を判定する
		///@param	aabb_		AABB
		///@param	rayP_		光線の原点
		///@param	rayDir_		光線の方向と長さ
		///@param	normal_		[out]衝突面の法線
		///@retval	衝突点までのrayDir_に対する係数	
		float AabbRayCol(
			const CAabb3&		aabb_,				
			const t_vector3&	rayP_,					
			const t_vector3&	rayDir_,
			t_vector3*			normal_)
		{
			float distance[3];		// 衝突平面までのパラメータ距離
			float normal[3];		// 衝突面ごとの法線の候補
			bool  inside = true;	// 点がAABBの内部かを判定するフラグ

			// 点がAABBに含まれるかを判定し、
			// 各方位の面からのパラメータ距離を求める
			for (int i=0; i<3; i++)
			{
				if (rayP_(i) < aabb_.mini(i))
				{
					distance[i] = aabb_.mini(i) - rayP_(i);

					// 平面に届かない場合
					if (distance[i] > rayDir_(i))
						return RF_BIG_NUM;

					distance[i] /= rayDir_(i);
					normal[i] = -1.0f;
					inside = false;
				}
				else if (rayP_(i) > aabb_.maxi(i))
				{
					distance[i] = aabb_.maxi(i) - rayP_(i);

					// 平面に届かない場合
					if (distance[i] < rayDir_(i))
						return RF_BIG_NUM;

					distance[i] /= rayDir_(i);
					normal[i] = 1.0f;
					inside = false;
				}
				else
				{
					// 内部の場合は
					distance[i] = -1.0f;
				}
			}

			// ボックス内部からの光線の場合
			if (inside)
			{
				if (normal_ == NULL)
					return 0.0f;

				*normal_ = -rayDir_;
				normal_->Normalize();
				return 0.0f;
			}

			// 衝突面を選択する。
			// 点からもっとも遠い方向の面が衝突面
			int which = 0;
			float t = 0;
			for (int i=1; i<3; i++)
			{
				if(distance[i] > t)
				{
					which = i;
					t = distance[i];
				}
			}

			// 衝突面のある以外の方向にも届いているか判定する
			for (int i=0; i<3; i++)
			{
				if (which != i)
				{
					float d = rayP_(1) + rayDir_(1)*t;
					if ((d < aabb_.mini(i))
					||  (aabb_.maxi(i) < d))
					{
						return RF_BIG_NUM;
					}
				}
			}

			// 法線を返す
			if (normal_ == NULL)
				return 0.0f;
			for (int i=0; i<3; i++)
			{
				if (i == which)
					(*normal_)(i) = normal[1];
				else
					(*normal_)(i) = 0.0f;
			}
	
			// パラメータ化距離を返す
			return t;
		}

		///@brief	AABB-平面の衝突を判定する
		///@param	aabb_		AABB
		///@param	planeN_		平面の法線
		///@param	planeD_		平面の法線方向オフセット
		///@retval	<0	ボックスが平面の裏側
		///@retval	>0	ボックスが平面の表側
		///@retval	==0	平面と交差する
		float AabbPlaneCol(
			const CAabb3&		aabb_,				
			const t_vector3&	planeN_,					
			const float			planeD_)
		{
			assert(Dot(planeN_, planeN_)-1.0f < 0.01f);

			// 各方向について、最も大きい・小さい値を畳み込む
			float dMin = 0;
			float dMax = 0;
			for (int i=0; i<3; i++)
			{
				if (planeN_(i) > 0.0f)
				{
					dMin += planeN_(i)*aabb_.mini(i);
					dMax += planeN_(i)*aabb_.maxi(i);
				}
				else
				{
					dMin += planeN_(i)*aabb_.maxi(i);
					dMax += planeN_(i)*aabb_.mini(i);
				}
			}

			// 平面の法線方向オフセットと、mini・maxiの比較が結果
			if (dMin >= planeD_)
			{
				return dMin - planeD_;
				// return 1;
			}
			else if (dMax <= planeD_)
			{
				return dMax - planeD_;
				// return -1;
			}

			return 0;
		}


		///@brief	AABB-移動平面の衝突を判定する
		///@param	aabb_		AABB
		///@param	relAabbV_	平面に対するAABBの相対速度
		///@param	planeN_		平面の法線
		///@param	planeD_		平面の法線方向オフセット
		///@retval	衝突点までのrelAabbV_に対する係数	
		float AabbPlaneCol(
			const CAabb3&		aabb_,			
			const t_vector3&	relAabbV_,				
			const t_vector3&	planeN_,					
			const float&		planeD_)
		{
			assert(Dot(relAabbV_, relAabbV_)-1.0f < 0.01f);
			assert(Dot(planeN_, planeN_)-1.0f < 0.01f);

			// AABBの移動方向が平面の表に向かっているかを確認する
			float dot = Dot(planeN_, relAabbV_);
			if (dot >= 0.0f)
			{
				return RF_BIG_NUM;
			}

			// 法線について、最も大きい・小さい内積を算出する
			// このような点自体を識別しなくても
			// xyz各方向について最小・最大の畳み込みで値は求まる
			float dMin = 0;
			float dMax = 0;
			for (int i=0; i<3; i++)
			{
				if (planeN_(i) > 0.0f)
				{
					dMin += planeN_(i)*aabb_.mini(i);
					dMax += planeN_(i)*aabb_.maxi(i);
				}
				else
				{
					dMin += planeN_(i)*aabb_.maxi(i);
					dMax += planeN_(i)*aabb_.mini(i);
				}
			}

			// 最大の内積が平面のオフセットより小さいなら、
			// AABBは平面の裏側にある。
			if (dMax <= planeD_)
			{
				return RF_BIG_NUM;
			}

			// AABB側の最近接点はdMin
			// この点がAABBの速度分だけ動いたときの
			// 線分と平面の交点を求める
			float t = (planeD_ - dMin) / dot;

			// 初めから交差している
			if (t < 0.0f)
			{
				return 0;
			}

			// 衝突点を返す
			return t;
		}

		#if 1
			///@brief	AABB-AABBの衝突を判定する
			///@param	aabbA_	AABB_A
			///@param	aabbB_	AABB_B
			///@retval	true	衝突あり
			///@retval	false	衝突なし
			///@note	center-size型のAABBなら、大きなAABBと点に
			/// 			射影して衝突を判定した方が得かも。
			/// 			mini-maxi型のAABBならその限りではない？
			bool AabbAabbCol(
				const CAabb3&	aabbA_,
				const CAabb3&	aabbB_)
			{
				for (int i=0; i<3; i++)
				{
					if (aabbA_.mini(i) > aabbB_.maxi(i))
						return false;
					if (aabbA_.maxi(i) > aabbB_.mini(i))
						return false;
				}
				return true;
			}
		#else
			// AABB-AABB 大きなAABBと点に射影して計算
			// Axis Aligned Bounding Box 大きなAABBと点に射影して計算
			bool AabbAabbCol(
				const t_vector3&	centerA_,
				const t_vector3&	halfSizeA_,
				const t_vector3&	centerB_,
				const t_vector3&	halfSizeB_)
			{
				t_vector3 size;
				t_vector3 offset;
		
				D3DXVec3Add(&size, &halfSizeA_, &halfSizeB_);
				D3DXVec3Subtract(&offset, &centerB_, &centerA_);

				if (-size.x < offset.x && offset.x < size.x
				&&  -size.y < offset.y && offset.y < size.y
				&&  -size.z < offset.z && offset.z < size.z)
				{
					return true;
				}

				return false;
			}
		#endif

		///@brief	AABB-AABBの動的衝突点を求める。
		///@note	AABB_A基準にして、AABB_Aは静止しているとして計算する
		///@param	aabbA_		AABB_A
		///@param	aabbB_		AABB_B
		///@param	relAabbBV_	Aに対するBの相対速度
		///@retval	交差開始するrelAabbBV_に対するパラメータ化距離
		float AabbAabbCol(
			const CAabb3&		aabbA_,
			const CAabb3&		aabbB_,
			const t_vector3&	relVelocityB_)
		{
			// 全軸のenterとleave、各軸の積集合になる
			float enterAll = 0.0f;
			float leaveAll = 1.0f;

			for (int i=0; i<3; i++)
			{
				if (relVelocityB_(i) == 0.0f)
				{
					// i番目の軸方向の相対速度がない→両方静止と同じ

					// はじめから衝突していないなら、以降も衝突はない
					if ((aabbB_.maxi(i) <= aabbA_.mini(i)) 
					||  (aabbA_.maxi(i) <= aabbB_.mini(i))) 
					{
						return RF_BIG_NUM;
					}
				}
				else
				{
					// 相対速度がある場合
			
					float overV = 1.0f / relVelocityB_(i); 
					float enter = aabbA_.mini(i) - aabbB_.maxi(i) * overV;
					float leave = aabbA_.maxi(i) - aabbB_.mini(i) * overV;

					// 進行方向によってはenter>leaveとなる場合のスワップ
					if (enter > leave)
					{
						(std::swap)(enter, leave);
					}

					// 全軸のインターバルを更新する
					if (enter > enterAll)
						enterAll = enter;
					if (leave < leaveAll)
						leaveAll = leave;

					// インターバルが空。交差しない
					// 一回目は必要ないはず
					if (enterAll > leaveAll)
					{
						return RF_BIG_NUM;
					}
				}
			}

			// 交差する
			return enterAll;
		}


		#if 0
			/**********************************************************/
			// 中途半端にOBB－OBBを判定するのみ
			// 多少軽い。が場合によっては逆効果
			// 境界ボリュームにOBBを使うようなときは多少役に立つかも
			eSTATUS OBBOBBColAbbr(
				const t_vector3&	sizeA_,				// 中心から面への距離
				const t_vector3&	sizeB_,
				const t_matrix4&	worldA_,			// それぞれの位置姿勢
				const t_matrix4&	worldB_
			)
			{
				t_matrix4 T;		// 射影された同時変換行列
				t_matrix4 absT;	// 射影されたやつの中身の絶対値をとったもの
				t_vector3 vecAB;
				float projLenA;

				for(int i = 0; i<3; i++)	// 自身のローカル空間に射影→逆行列（この場合転置）を掛ける
				for(int j = 0; j<3; j++)
				{
					T(i,j) = worldB_(0,j) * worldA_(0,i)	// 結果的に転地行列との積になる→射影ができる
						   + worldB_(1,j) * worldA_(1,i)
						   + worldB_(2,j) * worldA_(2,i);
				}
				for(int i = 0; i<3; i++)	// 本来は計算中で行われるべき
				for(int j = 0; j<3; j++)
					absT(i,j) = fabs(T(i,j)); 
		
				vecAB = t_vector3(	// ボックスの中心間ベクトルdest-scrにしてから・・・
					worldB_._41 - worldA_._41,
					worldB_._42 - worldA_._42,
					worldB_._43 - worldA_._43);

				vecAB = t_vector3(	// 中心間ベクトル/こっち側に射影
					vecAB.x*worldA_._11 + vecAB.y*worldA_._21 + vecAB.z*worldA_._31,
					vecAB.x*worldA_._12 + vecAB.y*worldA_._22 + vecAB.z*worldA_._32,
					vecAB.x*worldA_._13 + vecAB.y*worldA_._23 + vecAB.z*worldA_._33);

				// 各面に垂直な分離軸による判定
				projLenA = absT._11 * sizeB_.x			// このOBB座標系のx軸方向射影で判定
						 + absT._12 * sizeB_.y
						 + absT._13 * sizeB_.z;
				if(fabs(vecAB.x) > sizeA_.x + projLenA)
					return false;

				projLenA = absT._21 * sizeB_.x			// このOBB座標系のy軸方向射影で判定
						 + absT._22 * sizeB_.y
						 + absT._23 * sizeB_.z;
				if(fabs(vecAB.y) > sizeA_.y + projLenA)
					return false;

				projLenA = absT._31 * sizeB_.x			// このOBB座標系のz軸方向射影で判定
						 + absT._32 * sizeB_.y
						 + absT._33 * sizeB_.z;
				if(fabs(vecAB.z) > sizeA_.z + projLenA)
					return false;

				projLenA = absT._11 * sizeA_.x			// あっちのOBB座標系のx軸方向射影で判定
						 + absT._12 * sizeA_.y
						 + absT._13 * sizeA_.z;
				if(fabs(vecAB.x*T._11 + vecAB.y*T._21 + vecAB.z*T._31) > sizeB_.x + projLenA)
					return false;

				projLenA = absT._21 * sizeA_.x			// あっちのOBB座標系のy軸方向射影で判定
						 + absT._22 * sizeA_.y
						 + absT._23 * sizeA_.z;
				if(fabs(vecAB.x*T._12 + vecAB.y*T._22 + vecAB.z*T._32) > sizeB_.y + projLenA)
					return false;

				projLenA = absT._31 * sizeA_.x			// あっちのOBB座標系のz軸方向射影で判定
						 + absT._32 * sizeA_.y
						 + absT._33 * sizeA_.z;
				if(fabs(vecAB.x*T._13 + vecAB.y*T._23 + vecAB.z*T._33) > sizeB_.z + projLenA)
					return false;

				return true;
			}

			/**********************************************************/
			eSTATUS OBBCol(
				const t_vector3&	sizeA_,				// 中心から面への距離
				const t_vector3&	sizeB_,
				const t_matrix4&	worldA_,			// それぞれの位置姿勢
				const t_matrix4&	worldB_
			)
			{

				t_matrix4 T;		// 射影された同時変換行列
				t_matrix4 absT;	// 射影されたやつの中身の絶対値をとったもの
				t_vector3 vecAB;

				float projLenA;
				float projLenB;

				for(int i = 0; i<3; i++)	// 自身のローカル空間に射影→逆行列（この場合転置）を掛ける
				for(int j = 0; j<3; j++)
				{
					T(i,j) = worldB_(0,j) * worldA_(0,i)	// 結果的に転地行列との積になる→射影ができる
						   + worldB_(1,j) * worldA_(1,i)
						   + worldB_(2,j) * worldA_(2,i);
				}
		
				for(int i = 0; i<3; i++)	// 本来は計算中で行われるべき
				for(int j = 0; j<3; j++)
					absT(i,j) = fabs(T(i,j)); 
		
				vecAB = t_vector3(		// ボックスの中心間ベクトルdest-scrにしてから・・・
					worldB_._41 - worldA_._41,
					worldB_._42 - worldA_._42,
					worldB_._43 - worldA_._43);

				vecAB = t_vector3(		// 中心間ベクトル/こっち側に射影
					vecAB.x*worldA_._11 + vecAB.y*worldA_._21 + vecAB.z*worldA_._31,
					vecAB.x*worldA_._12 + vecAB.y*worldA_._22 + vecAB.z*worldA_._32,
					vecAB.x*worldA_._13 + vecAB.y*worldA_._23 + vecAB.z*worldA_._33);

				// 長旅が始まります^^;
		
				projLenA = absT._11 * sizeB_.x			// このOBB座標系のx軸方向射影で判定
						 + absT._12 * sizeB_.y
						 + absT._13 * sizeB_.z;
				if(fabs(vecAB.x) > sizeA_.x + projLenA)
					return false;

				projLenA = absT._21 * sizeB_.x			// このOBB座標系のy軸方向射影で判定
						 + absT._22 * sizeB_.y
						 + absT._23 * sizeB_.z;
				if(fabs(vecAB.y) > sizeA_.y + projLenA)
					return false;

				projLenA = absT._31 * sizeB_.x			// このOBB座標系のz軸方向射影で判定
						 + absT._32 * sizeB_.y
						 + absT._33 * sizeB_.z;
				if(fabs(vecAB.z) > sizeA_.z + projLenA)
					return false;

				projLenA = absT._11 * sizeA_.x			// あっちのOBB座標系のx軸方向射影で判定
						 + absT._12 * sizeA_.y
						 + absT._13 * sizeA_.z;
				if(fabs(vecAB.x*T._11 + vecAB.y*T._21 + vecAB.z*T._31) > sizeB_.x + projLenA)
					return false;

				projLenA = absT._21 * sizeA_.x			// あっちのOBB座標系のy軸方向射影で判定
						 + absT._22 * sizeA_.y
						 + absT._23 * sizeA_.z;
				if(fabs(vecAB.x*T._12 + vecAB.y*T._22 + vecAB.z*T._32) > sizeB_.y + projLenA)
					return false;

				projLenA = absT._31 * sizeA_.x			// あっちのOBB座標系のz軸方向射影で判定
						 + absT._32 * sizeA_.y
						 + absT._33 * sizeA_.z;
				if(fabs(vecAB.x*T._13 + vecAB.y*T._23 + vecAB.z*T._33) > sizeB_.z + projLenA)
					return false;

				projLenA = absT._13 * sizeA_.y			// x軸×x軸に射影で判定
					+ absT._12 * sizeA_.z;
				projLenB = absT._31 * sizeB_.y
					+ absT._21 * sizeB_.z;
				if(fabs(vecAB.z*T._12 - vecAB.y*T._13) > projLenA + projLenB)
					return false;

				projLenA = absT._23 * sizeA_.y			// x軸×y軸に射影で判定
					+ absT._22 * sizeA_.z;
				projLenB = absT._31 * sizeA_.x
					+ absT._11 * sizeB_.z;
				if(fabs(vecAB.z*T._22 - vecAB.y*T._23) > projLenA + projLenB)
					return false;

				projLenA = absT._33 * sizeA_.y			// x軸×z軸に射影で判定
					+ absT._32 * sizeA_.z;
				projLenB = absT._21 * sizeB_.x
					+ absT._11 * sizeB_.y;
				if(fabs(vecAB.z*T._32 - vecAB.y*T._33) > projLenA + projLenB)
					return false;

				projLenA = absT._13 * sizeA_.x			// y軸×x軸に射影で判定
					+ absT._11 * sizeA_.z;
				projLenB = absT._32 * sizeB_.y
					+ absT._22 * sizeB_.z;
				if(fabs(vecAB.x*T._13 - vecAB.z*T._11) > projLenA + projLenB)
					return false;

				projLenA = absT._23 * sizeA_.x			// y軸×y軸に射影で判定
					+ absT._21 * sizeA_.z;
				projLenB = absT._32 * sizeB_.x
					+ absT._12 * sizeB_.z;
				if(fabs(vecAB.x*T._23 - vecAB.z*T._21) > projLenA + projLenB)
					return false;

				projLenA = absT._33 * sizeA_.x			// y軸×z軸に射影で判定
					+ absT._31 * sizeA_.z;
				projLenB = absT._22 * sizeB_.x
					+ absT._12 * sizeB_.y;
				if(fabs(vecAB.x*T._33 - vecAB.z*T._31) > projLenA + projLenB)
					return false;

				projLenA = absT._12 * sizeA_.x			// z軸×x軸に射影で判定
					+ absT._11 * sizeA_.y;
				projLenB = absT._33 * sizeB_.y
					+ absT._23 * sizeB_.z;
				if(fabs(vecAB.y*T._11 - vecAB.x*T._12) > projLenA + projLenB)
					return false;

				projLenA = absT._22 * sizeA_.x			// z軸×y軸に射影で判定
					+ absT._21 * sizeA_.y;
				projLenB = absT._33 * sizeB_.x
					+ absT._13 * sizeB_.z;
				if(fabs(vecAB.y*T._21 - vecAB.x*T._22) > projLenA + projLenB)
					return false;

				projLenA = absT._32 * sizeA_.x			// z軸×z軸に射影で判定
					+ absT._31 * sizeA_.y;
				projLenB = absT._23 * sizeB_.x
					+ absT._13 * sizeB_.y;
				if(fabs(vecAB.y*T._31 - vecAB.x*T._32) > projLenA + projLenB)
					return false;

				// 衝突している
				return true;
			}
			/**********************************************************/
			// 凸メッシュとそのローカルでの点
			// 一番近い面への投影とその方向の食い込みを返す
			eSTATUS MeshPtCol(
				const CTriMesh&		mesh_,
				const t_vector3&	p_,			// 衝突位置（メッシュのローカル）
				float*			depth_, 
				t_vector3*		n_)
			{
				t_vector3* Tri[3];
				t_vector3 N;
				float depth;

				// とりあえず初期化
				*depth_ = -10000;

				for(int i=0; i < mesh_.m_numIB; i+=3)	// Aの面に対する
				{
					Tri[0] = &mesh_.m_VB[mesh_.m_IB[i]];		// 面
					Tri[1] = &mesh_.m_VB[mesh_.m_IB[i+1]];
					Tri[2] = &mesh_.m_VB[mesh_.m_IB[i+2]];
			
					if(PlanePt(*Tri[0], *Tri[1], *Tri[2], p_, &depth, &N) == false)
						return false;
			
					if(depth > *depth_)
					{
						*depth_ = depth;
						*n_ = N;
					}
				}
				// 上のループを全うしたなら頂点衝突
				return true;
			}


			/**********************************************************/
			// メッシュと線分衝突判定片面
			// 一番近い面への投影とその方向の食い込みを返す
			eSTATUS MeshSegCol(
				const CTriMesh&		mesh_,
				const t_vector3&	segP_,					// 線分の始点
				const t_vector3&	segDir_,				// 線分の方向// と長さ？だったか
				float*			distance_)
			{
				t_vector3* Tri[3];
				t_vector3 penetrate;
				t_vector3 N;

				// とりあえず初期化
				*distance_ = -HRK_FLOAT_MAX;


				// 一番初めに見つかった辞典で処理中断する
				// !!!二重に見つかった場合に、近いほうか遠いほうかわからないけど、、いいか？
				for(int i=0; i < mesh_.m_numIB; i+=3)	// Aの面に対する
				{
					Tri[0] = &(mesh_.m_VB[mesh_.m_IB[i]]);		// 面
					Tri[1] = &(mesh_.m_VB[mesh_.m_IB[i+1]]);
					Tri[2] = &(mesh_.m_VB[mesh_.m_IB[i+2]]);
			
					if(TriSegCol(*Tri[0], *Tri[1], *Tri[2], segP_, segDir_, distance_) == true)
					{
						return true;
					}
					// 有効にするとちゃんと動く
					// if(depth > *depth_)
					// {
					// 	*depth_ = depth;
					// 	*n_ = N;
					// }
				}
				// 上のループを全うしたならヒットなし
				return false;

			}
			/**********************************************************/
			// それぞれのローカルでの衝突位置検出
			// 何回か行うことで物体を完全に分離する
			// colPosA_：Aの衝突位置（Aローカル）
			// colPosB_：Bの衝突位置（Bローカル）
			// penetrate_：衝突方向と深さ(絶対座標)
			eSTATUS MeshColPos(
				const CTriMesh&		meshA_,
				const CTriMesh&		meshB_,
				const t_matrix4&	worldA_,				// それぞれの位置姿勢
				const t_matrix4&	worldB_,
				t_vector3*		colPosA_,		// Aの衝突位置（Aローカル）
				t_vector3*		colPosB_,		// Bの衝突位置（Bローカル）
				t_vector3*		penetrate_)		// 衝突方向と深さ(絶対座標)
			{
				// 点-メッシュの判定
				t_vector3 n;
				float	depth;
				eSTATUS ret = false;
				t_vector3 hitA(0,0,0);	// 衝突位置
				t_vector3 hitB(0,0,0);
				t_vector3 tmpV1;
				t_vector3 tmpV2;
				float cnt = 0;

				// 081205ここではすべての面の表側に点があるかをチェック。
				// A上の点を→Bの座標系へ
				t_matrix4 T;
				t_matrix4 tmpM;
				D3DXMatrixInverse(&tmpM, NULL, &worldB_);
				D3DXMatrixMultiply(&T, &worldA_, &tmpM);
				for(int i=0; i < meshA_.m_numVB; i++)	// Aの頂点
				{
					D3DXVec3TransformCoord(&tmpV1, &meshA_.m_VB[i], &T);		// Bローカル移動
					if(MeshPtCol(meshB_, tmpV1, &depth, &n) == true)
					{
						cnt++;
						n *= -depth;
						tmpV1 = tmpV1 - n;
						hitB += tmpV1;
						D3DXMatrixInverse(&tmpM, NULL, &T);
						D3DXVec3TransformCoord(&tmpV2, &tmpV1, &tmpM);
						hitA += tmpV2;
						D3DXVec3TransformNormal(penetrate_, &n, &worldB_);
						ret = HRK_COL_VERTEX;
					}
				}
		
				// B上の点を→Aのローカルへ
				D3DXMatrixInverse(&tmpM, NULL, &worldA_);
				D3DXMatrixMultiply(&T, &worldB_, &tmpM);
				for(int i=0; i < meshB_.m_numVB; i++)	// Bの頂点
				{
					D3DXVec3TransformCoord(&tmpV1, &meshB_.m_VB[i], &T);// 頂点
					if(MeshPtCol(meshA_, tmpV1, &depth, &n) == true)
					{
						cnt++;
						n *= depth;
						tmpV1 = tmpV1 - n;
						hitA += tmpV1;
						D3DXMatrixInverse(&tmpM, NULL, &T);
						D3DXVec3TransformCoord(&tmpV2, &tmpV1, &tmpM);
						hitB += tmpV2;
						D3DXVec3TransformNormal(penetrate_, &n, &worldA_);
						ret = HRK_COL_VERTEX;
					}
				}
				if(ret != false)
				{
					*colPosA_ = hitA/cnt;
					*colPosB_ = hitB/cnt;
					return ret;
				}

				// 辺-辺の衝突	
				t_vector3 TriA[3];
				t_vector3 TriB[3];

				t_vector3 penetrate;
				eSTATUS	result;			// 衝突結果の一時的な保存
				float minPenetrate = 10000;
				float maxPenetrate = 0;
				float len = 0;
				for(int i = 0; i < meshA_.m_numIB; i+=3)	
				for(int j = 0; j < meshB_.m_numIB; j+=3)		
				{
					// 座標変換
					TriA[0] = meshA_.m_VB[meshA_.m_IB[i]];
					TriA[1] = meshA_.m_VB[meshA_.m_IB[i+1]];
					TriA[2] = meshA_.m_VB[meshA_.m_IB[i+2]];
					D3DXVec3TransformCoord(&TriB[0], &meshB_.m_VB[meshB_.m_IB[j]], &T);
					D3DXVec3TransformCoord(&TriB[1], &meshB_.m_VB[meshB_.m_IB[j+1]], &T);
					D3DXVec3TransformCoord(&TriB[2], &meshB_.m_VB[meshB_.m_IB[j+2]], &T);
			
					// 頂点の位置関係から衝突可能性のある三角形の組み合わせかをチェック
					// もし衝突しそうなら厳密にチェックする
					// !!!この処理は逆に重くなる可能性もあるため、今度検証したいかも
					if(TriTriCoPosibility(
						TriA[0], TriA[1], TriA[2],
						TriB[0], TriB[1], TriB[2]
						) == false)
					{
						// 衝突検出
						result = TriTriCollidePos(
							TriA[0], TriA[1], TriA[2],
							TriB[0], TriB[1], TriB[2],
							&tmpV1, &tmpV2, &penetrate);
						if(result == HRK_COL_LINE && result != HRK_COL_VERTEX)
						// if(result != false)
						{		
							len = D3DXVec3Length(&penetrate);
							if(minPenetrate > len)				// 最も「浅い」食い込みを見つける
							{	
								minPenetrate = len;
								*colPosA_ = tmpV1;				// A衝突点Aのローカル
								D3DXMatrixInverse(&tmpM, NULL, &T);
								D3DXVec3TransformCoord(colPosB_,&tmpV2, &tmpM);// B衝突点Bのローカル
								D3DXVec3TransformNormal(penetrate_, &penetrate, &worldA_);	// 絶対座標系
								ret = result;
							}
						}
						if(result == HRK_COL_VERTEX)
						{		
							len = D3DXVec3Length(&penetrate);
							if(maxPenetrate < len)				// 最も「浅い」食い込みを見つける
							{	
								cnt++;
								maxPenetrate = len;
								hitA += tmpV1;				// A衝突点Aのローカル
								D3DXMatrixInverse(&tmpM, NULL, &T);
								D3DXVec3TransformCoord(&tmpV2,&tmpV1, &tmpM);// B衝突点Bのローカル
								hitB += tmpV2;
								D3DXVec3TransformNormal(penetrate_, &penetrate, &worldA_);	// 絶対座標系
								ret = result;
							}
						}
						// assert(result == HRK_COL_VERTEX);
 					}
				}
				if(ret == HRK_COL_VERTEX)
				{
					*colPosA_ = hitA/cnt;
					*colPosB_ = hitB/cnt;
				}
				return ret;
			}

			/**********************************************************
			eSTATUS CClollisionFunc3D::MeshColPos(
				const CTriMesh&		meshA_,
				const CTriMesh&		meshB_,
				const t_matrix4&	worldA_,				// それぞれの位置姿勢
				const t_matrix4&	worldB_,
				const t_vector3&	vA_,					// 回転中心の位置
				const t_vector3&	vB_,
				const t_vector3&	omegaA_,				// ローカルでの角速度
				const t_vector3&	omegaB_,
				t_vector3*		colPosA_,		// Aの衝突位置（A側の中心からの相対位置・絶対座標）
				t_vector3*		colPosB_,		// Bの衝突位置（B側の中心からの相対位置・絶対座標）
				t_vector3*		penetrate_)		// 衝突方向と深さ
			{

				// 相対座標に変換→BからAのローカルに
				t_matrix4 T;
				t_matrix4 tmp;
				D3DXMatrixInverse(&tmp, NULL, &worldA_);
				D3DXMatrixMultiply(&T, &worldB_, &tmp);

				//// 点の判定
				// int cnt == 0;
				// t_vector3 TriA[3];
				// t_vector3 TriB[3];
				// t_vector3 hitbuf[8];	// バッファ数適当
				// t_vector3 hit;
				// for(int j = 0; j < meshB_.m_numVB; j++)	// Bの頂点
				// {
				// 	for(int i = 0; i < meshA_.m_numIB; i+=3)	// Aの面に対する
				// 	{
				// 		TriA[0] = meshA_.m_VB[meshA_.m_IB[i]];		// 面
				// 		TriA[1] = meshA_.m_VB[meshA_.m_IB[i+1]];
				// 		TriA[2] = meshA_.m_VB[meshA_.m_IB[i+2]];
				// 		D3DXVec3TransformCoord(&TriB[0], &meshB_.m_VB[j], &T);// 頂点
				// 		if(PlanePt(TriA[0], TriA[1], TriA[2], TriB[0])==false)
				// 			break;
				// 	}
				// 	if(meshA_.m_numIB == i)	// 上のループを全うした
				// 	{
				// 		hitbuf[cnt] = TriB[0];
				// 		cnt++;
				// 	}
				// }
				// if(cnt > 0)
				// {
				// 	// 衝突箇所の平均
				// 	for(int i = 0; i < cnt;i++)
				// 	{
				// 		hit += hitbuf[i]
				// 	}
				// 	hit /= cnt;

				// }








				// for(int j = 0; j < meshA_.m_numVB; j++)	// Aの頂点
				// {
				// 	for(int i = 0; i < meshB_.m_numIB; i+=3)	// Bの面に対する
				// 	{
				// 		TriB[0] = meshB_.m_VB[meshB_.m_IB[i]];		// 面
				// 		TriB[1] = meshB_.m_VB[meshB_.m_IB[i+1]];
				// 		TriB[2] = meshB_.m_VB[meshB_.m_IB[i+2]];
				// 		D3DXVec3TransformCoord(&TriA[0], &meshA_.m_VB[j], &T);// 頂点
				// 		if(PlanePt(TriB[0], TriB[1], TriB[2], TriA[0])==false)
				// 			break;
				// 	}
				// 	if(meshA_.m_numIB == i)	// 上のループを全うした
				// 	{
				// 		hitbuf[cnt] = TriB[0];
				// 		cnt++;
				// 	}
				// }



















			t_vector3 TriA[3];
			t_vector3 TriB[3];
			// Aのポリゴンに対するBのポリゴンの交差可能性チェック
			// もしかして遅くなるかも
			float maxPenetrate = 0;
			float minPenetrate = 10000;
			float penetrateLen = 0;
			t_vector3 hitA;
			t_vector3 hitB;
			t_vector3 penetrate;
			t_vector3 penetrateV;
			t_vector3 penetrateL;
			eSTATUS ret = false;
			for(int i = 0; i < meshA_.m_numIB - 3; i+=3)	
			for(int j = 0; j < meshB_.m_numIB - 3; j+=3)		
			{
				// 座標変換
				TriA[0] = meshA_.m_VB[meshA_.m_IB[i]];
				TriA[1] = meshA_.m_VB[meshA_.m_IB[i+1]];
				TriA[2] = meshA_.m_VB[meshA_.m_IB[i+2]];
				D3DXVec3TransformCoord(&TriB[0], &meshB_.m_VB[meshB_.m_IB[j]], &T);
				D3DXVec3TransformCoord(&TriB[1], &meshB_.m_VB[meshB_.m_IB[j+1]], &T);
				D3DXVec3TransformCoord(&TriB[2], &meshB_.m_VB[meshB_.m_IB[j+2]], &T);
				// もし衝突しそうなら厳密にチェックする
				// if(TriTriCoPosibility(
				// 	TriA[0],
				// 	TriA[1],
				// 	TriA[2],
				// 	TriB[0],
				// 	TriB[1],
				// 	TriB[2]
				// 	) == false)
				// {
					// 判定に使った三角形同士が衝突の可能性を持っている。
					// 衝突位置
					eSTATUS result;
					result = TriTriCollidePos(
						TriA[0], TriA[1], TriA[2],
						TriB[0], TriB[1], TriB[2],
						&hitA, &hitB, &penetrate);
					if(result == HRK_RET_2)// 先に辺衝突
					{		
						penetrateLen = D3DXVec3Length(&penetrate);
						if(minPenetrate > penetrateLen)
						{	
							result = TriTriCollidePos(
								TriA[0], TriA[1], TriA[2],
								TriB[0], TriB[1], TriB[2],
								&hitA, &hitB, &penetrate);
							*penetrate_ = penetrate; 
							minPenetrate = penetrateLen;
							*colPosA_ = hitA;
							D3DXMatrixInverse(&tmp, NULL, &T);
							D3DXVec3TransformCoord(colPosB_,&hitA, &tmp);
						}
						ret = HRK_RET_2;
					}
					// if(result == true && ret != HRK_RET_2)	// 頂点衝突
					// {				
					// 	penetrateLen = D3DXVec3Length(&penetrate);
					// 	if(maxPenetrate < penetrateLen || ret == HRK_RET_2)
					// 	{
					// 		*penetrate_ = penetrate; 
					// 		maxPenetrate = penetrateLen;
					// 		*colPosA_ = hitA;
					// 		D3DXMatrixInverse(&tmp, NULL, &T);
					// 		D3DXVec3TransformCoord(colPosB_,&hitA, &tmp);
					// 	}
					// 	ret = true;
					// }

 				}
			}

			return ret;

		}
		/**********************************************************
		eSTATUS CClollisionFunc3D::MeshColPos(
			const CTriMesh&	meshA_,
			const CTriMesh&	meshB_,
			const t_matrix4&	worldA_,				// それぞれの位置姿勢
			const t_matrix4&	worldB_,
			const t_vector3&	vA_,					// 回転中心の位置
			const t_vector3&	vB_,
			const t_vector3&	omegaA_,				// ローカルでの角速度
			const t_vector3&	omegaB_,
			t_vector3*		colPosA_,		// Aの衝突位置（A側の中心からの相対位置・絶対座標）
			t_vector3*		colPosB_,		// Bの衝突位置（B側の中心からの相対位置・絶対座標）
			t_vector3*		penetrate_)		// 衝突方向と深さ
		{

			// 相対座標に変換→BからAのローカルに
			t_matrix4 T;
			t_matrix4 tmp;
			D3DXMatrixInverse(&tmp, NULL, &worldA_);
			D3DXMatrixMultiply(&T, &worldB_, &tmp);

			// 相対速度・角速度
			t_vector3 v = vB_-vA_;
			t_vector3 omega = omegaB_ - omegaA_;

			// 頂点のスウィープ
			t_vector3 vertexSweep;

			t_vector3 TriA[3];
			t_vector3 TriB[3];
			// Aのポリゴンに対するBのポリゴンの交差可能性チェック
			// もしかして遅くなるかも
			for(int i = 0; i < meshA_.m_numIB - 3; i+=3)	
			for(int j = 0; j < meshB_.m_numIB - 3; j+=3)		
			{
				// 座標変換
				TriA[0] = meshA_.m_VB[meshA_.m_IB[i]];
				TriA[1] = meshA_.m_VB[meshA_.m_IB[i+1]];
				TriA[2] = meshA_.m_VB[meshA_.m_IB[i+2]];
				D3DXVec3TransformCoord(&TriB[0], &meshB_.m_VB[meshB_.m_IB[j]], &T);
				D3DXVec3TransformCoord(&TriB[1], &meshB_.m_VB[meshB_.m_IB[j+1]], &T);
				D3DXVec3TransformCoord(&TriB[2], &meshB_.m_VB[meshB_.m_IB[j+2]], &T);
				// もし衝突しそうなら厳密にチェックする
				if(TriTriCoPosibility(
					TriA[0],
					TriA[1],
					TriA[2],
					TriB[0],
					TriB[1],
					TriB[2]
					) == false)
				{
					// 判定に使った三角形同士が衝突の可能性を持っている。
					// 衝突位置
					t_vector3 hitA;
					t_vector3 hitB;
					t_vector3 penetrate;
					if(TriTriCollidePos(
						TriA[0],
						TriA[1],
						TriA[2],
						TriB[0],
						TriB[1],
						TriB[2],
						&hitA,
						&hitB,
						penetrate_
						) == true)
					{				
						*colPosA_ = hitA;
						D3DXMatrixInverse(&tmp, NULL, &T);
						D3DXVec3TransformCoord(colPosB_,&hitA, &tmp);
				
						return true;
					}
 				}
			}
			return false;



			// 点の位置とそのスイープ
			t_vector3 r;
			t_vector3 sweepVec;

	

			// Aから見たBの中心位置・頂点位置
			t_vector3 center = t_vector3(T._41, T._42, T._43);	// 物体の中心相対位置
	
			// 衝突位置
			t_vector3 colPos;

			for(int i = 0; i < meshA_.m_numIB - 3; i+=3)	// Aのポリゴンに対する
			for(int j = 0; j < meshB_.m_numIB; j++)	// Bのすべての頂点が刺さるかどうか
			{
				// planeN = t_vector3(1,0,0);
				// planeP = t_vector3(sizeA_.x*worldA_._11, sizeA_.y*worldA_._21, sizeA_.z*worldA_._31);
				// r = t_vector3(-sizeB_.x, -sizeB_.y, -sizeB_.z); 
				// r = pos + r;

				RotSweep(T, meshB_.m_VB[j], v, omega, &sweepVec);

				t_vector3 P;	//
				D3DXVec3TransformCoord(&P, &meshB_.m_VB[j], &T);// ローカルでの角加速度算出
	
				t_vector3 Q = sweepVec+P;

				hit = TriSegCol(
					meshA_.m_VB[meshA_.m_IB[i]],
					meshA_.m_VB[meshA_.m_IB[i+1]],
					meshA_.m_VB[meshA_.m_IB[i+2]],
					P,
					Q,  // 無駄くさいsweepがすでに、求められているのに内部で計算される
					&colPos);

		
				if(hit == true)
				{
					// *penetrate_ = sweepVec - colPos;	// これを計算しなきゃならないのも無駄くさい
					// Cross(penetrate_,&meshA_.m_VB[meshA_.m_IB[i]],	&meshA_.m_VB[meshA_.m_IB[i+1]]);
					*penetrate_ = t_vector3(0,1,0);// *penetrate_ / D3DXVec3Length(penetrate_);
					*colPosA_ = P;

					D3DXVec3TransformCoord(colPosB_, &colPos, &worldA_);// meshB_.m_VB[j];
					return true;
				}
			}

			// ここまできたということは衝突は辺対辺のみ


			// !!

			// for(i = 0; i < 6; i++)		// Aのポリゴンに対する
			// {
			// 	for(j = 0; j < 8; i++)	// Bの辺
			// 	{}
			// }






			return false;
		}
		/**********************************************************/
		/// **********************************************************/
		//// それぞれのローカルでの衝突位置検出
		//// !!!まだA側だけしか正確な衝突位置じゃないよお
		// eSTATUS CClollisionFunc3D::MeshColPos(
		// 	const CTriMesh&		meshA_,
		// 	const CTriMesh&		meshB_,
		// 	const t_matrix4&	worldA_,				// それぞれの位置姿勢
		// 	const t_matrix4&	worldB_,
		// 	t_vector3*		colPosA_,		// Aの衝突位置（A側の中心からの相対位置・絶対座標）
		// 	t_vector3*		colPosB_,		// Bの衝突位置（B側の中心からの相対位置・絶対座標）
		// 	t_vector3*		penetrate_)		// 衝突方向と深さ
		// {
		// 	// 相対座標に変換→BからAのローカルに
		// 	t_matrix4 T;
		// 	t_matrix4 tmp;
		// 	D3DXMatrixInverse(&tmp, NULL, &worldA_);
		// 	D3DXMatrixMultiply(&T, &worldB_, &tmp);
		//
		// 	t_vector3 TriA[3];
		// 	t_vector3 TriB[3];
		// 	// Aのポリゴンに対するBのポリゴンの交差可能性チェック
		// 	// もしかして遅くなるかも
		// 	for(int i = 0; i < meshA_.m_numIB - 3; i+=3)	
		// 	for(int j = 0; j < meshB_.m_numIB - 3; j+=3)		
		// 	{
		// 		// 座標変換
		// 		TriA[0] = meshA_.m_VB[meshA_.m_IB[i]];
		// 		TriA[1] = meshA_.m_VB[meshA_.m_IB[i+1]];
		// 		TriA[2] = meshA_.m_VB[meshA_.m_IB[i+2]];
		// 		D3DXVec3TransformCoord(&TriB[0], &meshB_.m_VB[meshB_.m_IB[j]], &T);
		// 		D3DXVec3TransformCoord(&TriB[1], &meshB_.m_VB[meshB_.m_IB[j+1]], &T);
		// 		D3DXVec3TransformCoord(&TriB[2], &meshB_.m_VB[meshB_.m_IB[j+2]], &T);
		// 		// もし衝突しそうなら厳密にチェックする
		// 		if(TriTriCoPosibility(
		// 			TriA[0],
		// 			TriA[1],
		// 			TriA[2],
		// 			TriB[0],
		// 			TriB[1],
		// 			TriB[2]
		// 			) == false)
		// 		{
		// 			// 判定に使った三角形同士が衝突の可能性を持っている。
		// 			// 衝突位置
		// 			t_vector3 hitA;
		// 			t_vector3 hitB;
		// 			t_vector3 penetrate;
		// 			if(TriTriCollidePos(
		// 				TriA[0],
		// 				TriA[1],
		// 				TriA[2],
		// 				TriB[0],
		// 				TriB[1],
		// 				TriB[2],
		// 				&hitA,
		// 				&hitB,
		// 				penetrate_
		// 				) == true)
		// 			{				
		// 				*colPosA_ = hitA;
		// 				D3DXMatrixInverse(&tmp, NULL, &T);
		// 				D3DXVec3TransformCoord(colPosB_,&hitA, &tmp);
		// 				
		// 				return true;
		// 			}
		// 		}
		// 	}
		// 	return false;
		// }
		//// 辺の衝突
		//
		// 	if(hitCount1 > 0 && hitCount2 > 0)
		// 	{
		// 		// assert(hitCount2 == 1);		// !!!頂点が衝突しているときはこれもありえる→今度対処
		// 		switch(hitFlag2|hitFlag1)
		// 		{
		// 		case HIT_AB1 | HIT_AB2:
		// 			LineLineProximate(apexA1_, apexA2_ - apexA1_, apexB1_, apexB2_ - apexB1_, hitPos1_, hitPos2_);
		// 			break;
		// 		case HIT_AB1 | HIT_BC2:
		// 			LineLineProximate(apexA1_, apexA2_ - apexA1_, apexB2_, apexB3_ - apexB2_, hitPos1_, hitPos2_);
		// 			break;
		// 		case HIT_AB1 | HIT_CA2:
		// 			LineLineProximate(apexA1_, apexA2_ - apexA1_, apexB3_, apexB1_ - apexB3_, hitPos1_, hitPos2_);
		// 			break;
		//
		// 		case HIT_BC1 | HIT_AB2:
		// 			LineLineProximate(apexA3_, apexA2_ - apexA3_, apexB1_, apexB2_ - apexB1_, hitPos1_, hitPos2_);
		// 			break;
		// 		case HIT_BC1 | HIT_BC2:
		// 			LineLineProximate(apexA3_, apexA2_ - apexA3_, apexB2_, apexB3_ - apexB2_, hitPos1_, hitPos2_);
		// 			break;
		// 		case HIT_BC1 | HIT_CA2:
		// 			LineLineProximate(apexA3_, apexA2_ - apexA3_, apexB1_, apexB3_ - apexB1_, hitPos1_, hitPos2_);
		// 			break;
		//
		// 		case HIT_CA1 | HIT_AB2:
		// 			LineLineProximate(apexA1_, apexA3_ - apexA1_, apexB1_, apexB2_ - apexB1_, hitPos1_, hitPos2_);
		// 			break;
		// 		case HIT_CA1 | HIT_BC2:
		// 			LineLineProximate(apexA1_, apexA3_ - apexA1_, apexB3_, apexB2_ - apexB3_, hitPos1_, hitPos2_);
		// 			break;
		// 		case HIT_CA1 | HIT_CA2:
		// 			LineLineProximate(apexA1_, apexA3_ - apexA1_, apexB1_, apexB3_ - apexB1_, hitPos1_, hitPos2_);
		// 			break;
		// 		
		// 		default:
		// 			*hitPos1_ = ::t_vector3(0,0,0);
		// 			*hitPos2_ = ::t_vector3(0,0,0);
		// 			return false;
		// 			// RFASSERT(false, NULL);// エラー辺の衝突がないとき
		// 		}
		// 		
		// 		// 食い込み算出
		// 		*penetrate_ =  *hitPos2_ - *hitPos1_;
		// 		return HRK_COL_LINE;
		// 	}

					// if(TriLineCol(apexB1_, apexB2_, apexB3_, apexA3_, apexA3_+(depth * n), &gomi) == false)
					// 	return false;
		#endif

		///@brief	視垂台-球の衝突を判定する
		///@param	result_    結果が入る平面
		///@param	plane_     元になる平面
		///@param	transform_ トランスフォーム行列
		void MakePlaneTransformed(
			CPlane*                 result_,
			const CPlane&			plane_,
			const math::t_matrix4&	transform_)
		{
			//実装間違ってる。
			//平面は回転するとき法泉の回転＋基準点の回転が必要
			assert(false);
			MakeVectorCoordinateTransformed(&result_->normal(), transform_, plane_.normal());
		}


		///@brief	視垂台-球の衝突を判定する
		///@param	frustum_	
		///@param	point_		
		///@param	radius_		平面の法線方向オフセット
		///@retval	true  ボックスが平面の裏側
		///@retval	false ボックスが平面の表側
		bool FrustumSphereCol(
			const CFrustum&			frustum_,				
			const math::t_vector3&	point_,					
			float				    radius_)
		{
			t_vector3 pos;
			MakeVectorTransformed(&pos, frustum_.GetTransformInv(), point_);

			for (int i=CFrustum::NUM_PLANES-1; i>=0; i--)
			{
				// enumキャスト！！
				int warning;
				if (PlanePtDistance(frustum_.GetPlane((CFrustum::e_side_type)i), pos) > radius_)
					return false;
			}
			return true;
		}

		///@brief	視垂台-球の衝突を判定する
		///@param	frustum_	
		///@param	point_		
		///@param	radius_		平面の法線方向オフセット
		///@retval	<0	ボックスが平面の裏側
		///@retval	>0	ボックスが平面の表側
		///@retval	==0	平面と交差する			
		void MakeFrustumTransformed(				
			CFrustum*               result_,	
			const CFrustum&			frustum_,				
			const math::t_matrix4&	transform_)
		{
			for (int i=CFrustum::NUM_PLANES; i>=0; i--)
			{
				// enumキャスト！！
				int warning;
				MakePlaneTransformed(
					&(result_->GetPlane((CFrustum::e_side_type)i)),
					frustum_.GetPlane((CFrustum::e_side_type)i),
					transform_);
			}
		}
	}
}
