///
///@brief		プリミティブ衝突関連関数群
/// 					- 2D衝突判定
/// 						- 点(Pt)
/// 						- 円(Circle)
/// 						- 線(line)
/// 						- 線分(seg)
/// 						- 半直線/レイ(Ray)
/// 						- Axis Aligned Bounding Box(Aabb)
/// 					- 3D衝突判定
/// 						- 点(Pt)
/// 						- 球(Sphere)
/// 						- 線(line)
/// 						- 線分(seg)
/// 						- 半直線/レイ(Ray)
/// 						- 平面(Plane)
/// 						- 三角形(Tri)
/// 						- 四角形(Square)
/// 						- Axis Aligned Bounding Box(Aabb)
/// 						- Oriented Bounding Box(Obb)
///
///@attention	DirectX用に左手座標系である
///@auther		RockField
///@date		2010/04/29
///@file		collisiondetection.h
///

#ifndef _RF_COLLISION_DITECT_FUNCTION_H_
#define _RF_COLLISION_DITECT_FUNCTION_H_

#include "../math/index.h"
#include "boundingbox.h"
#include "plane.h"
#include "frustum.h"

///@brief rf
namespace rf
{
	///@brief collision
	namespace collision
	{
		///@name	definitions
		///@brief	項等行列、零ベクトル等の定義
		// @{

		/////@brief	グローバル関数
		// namespace util
		// {
			///@defgroup	intersection
			///@brief	交差、衝突判定
			// @{

			///@brief	点-点の距離を求める
			///@param	a_	点Aの座標
			///@param   b_	点Bの座標
			///@retval  点-点の距離
			float PtPtDistance(
				const math::t_vector2&	a_,
				const math::t_vector2&	b_);

			///@brief	円-円の衝突を判定する
			///@param	centerA_	円Aの中心座標
			///@param	radiusA_	円Aの半径
			///@param   centerB_	円Bの中心座標
			///@param   radiusB_	円Bの半径
			///@retval	true		衝突あり
			///@retval	false		衝突なし
			///@note	円の半径を足して円と点の衝突判定に射影する
			float CircleCircleCol(
				const math::t_vector2&	centerA_,
				float 					radiusA_,
				const math::t_vector2&	centerB_,
				float 					radiusB_);

			#if 0
				///@brief	線分-線分の最近接点
				///@param	segAP_		線Aの始点
				///@param	segADir_	線Aの終点
				///@param segBP_		線Bの始点
				///@param segBDir_	線Bの終点
				///@param	proximateA_	[out]線A上の最近接点
				///@param	proximateB_	[out]線B上の最近接点
				void SegSegCol(
					const math::t_vector2&	segAP_,
					const math::t_vector2&	segAQ_,
					const math::t_vector2&	segBP_,
					const math::t_vector2&	segBQ_,
					math::t_vector2*		intersection_);
			#endif

			///@brief	AABB-点の衝突を判定する
			///@param	aabb_		AABB
			///@param	p_			点の座標
			///@retval	true		衝突あり
			///@retval	false		衝突なし
			bool AabbPtCol(
				const CAabb2&			aabb_,
				const math::t_vector2&	p_);

			///@brief	AABB-点最近接点を取得する
			///@param	aabb_			AABB
			///@param	p_				点の座標
			///@param	proximate_		[out]最近接点
			void AabbPtProximate(
				const CAabb2&			aabb_,
				const math::t_vector2&	p_,
				math::t_vector2*		proximate_);

			///@brief	AABB-AABBの衝突を判定する
			///@param	aabbA_	AABB_A
			///@param	aabbB_	AABB_B
			///@retval	true		衝突あり
			///@retval	false		衝突なし
			bool AabbAabbCol(
				const CAabb2&	aabbA_,
				const CAabb2&	aabbB_);

			// @}
		// }
	}

	///@brief	3D衝突判定
	namespace collision
	{
		typedef enum {
			RF_COLLIDE_NONE   = 0,
			RF_COLLIDE_VERTEX,	// 頂点と平面の衝突
			RF_COLLIDE_EDGE,	// 辺と辺の衝突
			RF_COLLIDE_NORMAL,	// その他の衝突
		}e_collide_bits;

		/////@brief	グローバル関数
		// namespace util
		// {
			///@defgroup	intersection
			///@brief	交差、衝突判定
			// @{

			///@brief	回転体の位置と速度と角速度から、ある点の速度を求める
			///@param	world_		回転体のワールド変換行列
			///@param   v_			回転体の速度
			///@param	w_			回転体の角速度
			///@param	p_			速度を求めたい点の回転体ローカル座標
			///@param   sweepVec_	[out]ワールドでの点の速度
			void RotSweep(
				const math::t_matrix4&	world_,
				const math::t_vector3&	v_,
				const math::t_vector3&	w_,
				const math::t_vector3&	p_,
				math::t_vector3*		sweepVec_);

			///@brief	点-点の距離を求める
			///@param	a_	点Aの座標
			///@param   b_	点Bの座標
			///@retval  点-点の距離
			float PtPtDistance(
				const math::t_vector3&	a_,
				const math::t_vector3&	b_);

			///@brief	円-円の衝突を判定する
			///@param	centerA_	円Aの中心座標
			///@param	radiusA_	円Aの半径
			///@param   centerB_	円Bの中心座標
			///@param   radiusB_	円Bの半径
			///@retval  点-点の距離
			float SphereSphereCol(
				const math::t_vector3&	a_,
				float 					radiusA_,
				const math::t_vector3&	b_,
				float 					radiusB_);

			///@brief	点-線の距離を求める
			///@param	p_			点の座標
			///@param	lineP_		線上の任意の点
			///@param   lineDir_	正規化された線の方向
			///@retval  点-線の距離
			float LinePtDistance(
				const math::t_vector3&	p_,
				const math::t_vector3&	lineP_,
				const math::t_vector3&	lineDir_);

			///@brief	線上にある点p_最近接点を求める
			///@param	p_			点の座標
			///@param	lineP_		線上の任意の点
			///@param   lineDir_	線の方向
			///@param   proximate_	最近接点
			void LinePtProximate(
				const math::t_vector3&	p_,
				const math::t_vector3&	lineP_,
				const math::t_vector3&	lineDir_,
				math::t_vector3*		proximate_);

			///@brief	点-線分の距離を求める
			///@param	p_			点の座標
			///@param	segP_		線分の始点
			///@param   segQ_		線分の終点
			///@retval	点-線分の距離
			float SegPtDistance(
				const math::t_vector3&	p_,
				const math::t_vector3&	segP_,
				const math::t_vector3&	segQ_);

			///@brief	線分上にある点p_の最近接点を求める
			///@param	p_			点の座標
			///@param	segP_		線分の始点
			///@param   segQ_		線分の終点
			///@param   proximate_	[out]最近接点
			void SegPtProximate(
				const math::t_vector3&	segP_,
				const math::t_vector3&	segQ_,
				const math::t_vector3&	p_,
				math::t_vector3*		proximate_);

			///@brief	線-線の距離を求める
			///@param	segAP_		線Aの始点
			///@param	segADir_	線Aの終点
			///@param   segBP_		線Bの始点
			///@param   segBDir_	線Bの終点
			///@retval	線-線の距離
			float LineLineDistance(
				const math::t_vector3&	segAP_,
				const math::t_vector3&	segADir_,
				const math::t_vector3&	segBP_,
				const math::t_vector3&	segBDir_);

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
				const math::t_vector3&	lineAP_,
				const math::t_vector3&	lineADir_,
				const math::t_vector3&	lineBP_,
				const math::t_vector3&	lineBDir_,
				math::t_vector3*		proximateA_,
				math::t_vector3*		proximateB_);


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
			void SegSegProximate(
				const math::t_vector3&	segAP_,
				const math::t_vector3&	segAQ_,
				const math::t_vector3&	segBP_,
				const math::t_vector3&	segBQ_,
				math::t_vector3*		proximateA_,
				math::t_vector3*		proximateB_);

			///@brief	平面-点の位置関係を判定する
			///@param	planeP_		平面上の任意の点
			///@param	planeN_		平面の法線
			///@param   p_			点の座標
			///@retval	true		表側
			///@retval	false		裏側
			bool PlanePtClassifyInside(
				const math::t_vector3&	planeP_,
				const math::t_vector3&	planeN_,
				const math::t_vector3&	p_);

			///@brief	平面-点の位置関係を判定する
			///@param	apex1_		平面上の任意の点A
			///@param	apex2_		平面上の任意の点B
			///@param	apex3_		平面上の任意の点C
			///@param   p_			点の座標
			///@retval	true		表側
			///@retval	false		裏側
			///@note	平面の裏表の定義は左手系に準じる
			bool PlanePtClassifyInside(
				const math::t_vector3&	apex1_,
				const math::t_vector3&	apex2_,
				const math::t_vector3&	apex3_,
				const math::t_vector3&	p_);

			///@brief	平面と点の符号付距離を求める
			///@param	plane_		平面
			///@param	p_			平面の法線
			///@param   depth_		[out]平面と点の符号付距離
			///@retval  平面と点の符号付距離、負が表側
			///@note	平面の裏表の定義は左手系に準じる
			float PlanePtDistance(
				const CPlane&			plane_,
				const math::t_vector3&	p_);

			///@brief	平面と点の符号付距離を求める
			///@param	planeP_		平面上の任意の点
			///@param	planeN_		平面の法線
			///@param	p_			平面の法線
			///@param   depth_		[out]平面と点の符号付距離
			///@retval  平面と点の符号付距離、負が表側
			///@note	平面の裏表の定義は左手系に準じる
			///@note	本来はPの位置が分からなくても距離が求まるので少し損
			float PlanePtDistance(
				const math::t_vector3&	planeP_,
				const math::t_vector3&	planeN_,
				const math::t_vector3&	p_);

			///@brief	平面と点の符号付距離を求める
			///@param	apex1_		平面上の任意の点A
			///@param	apex2_		平面上の任意の点B
			///@param	apex3_		平面上の任意の点C
			///@param	p_			平面の法線
			///@param   depth_		[out]平面と点の符号付距離
			///@param   normal_		[out]平面の法線
			///@retval  平面と点の符号付距離、負が表側
			///@note	平面の裏表の定義は左手系に準じる
			float PlanePtDistance(
				const math::t_vector3&	apex1_,
				const math::t_vector3&	apex2_,
				const math::t_vector3&	apex3_,
				const math::t_vector3&	p_,
				float*					depth_,
				math::t_vector3*		normal_);

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
				const math::t_vector3&	planeP_,
				const math::t_vector3&	planeN_,
				const math::t_vector3&	segP_,
				const math::t_vector3&	segQ_,
				math::t_vector3*		intersection_ = NULL);

			///@brief	平面と半直線の衝突を判定する
			///@param	planeP_			平面上の任意の点
			///@param	planeN_			平面の法線
			///@param	rayP_,			半直線の始点
			///@param	rayDir_			半直線の長さと方向
			///@param   intersection_	[out]衝突点の座標、省略可
			///@retval	true			衝突あり
			///@retval	false			衝突なし
			bool PlaneRayCol(
				const math::t_vector3&	planeP_,
				const math::t_vector3&	planeN_,
				const math::t_vector3&	rayP_,
				const math::t_vector3&	rayVec_,
				math::t_vector3*		intersection_ = NULL);

			///@brief	両面三角-線の衝突位置を求める
			///@param	apex1_			三角形の頂点A
			///@param	apex2_			三角形の頂点B
			///@param	apex3_			三角形の頂点C
			///@param	lineP_			線上の任意の点
			///@param	lineDir_		線の方向
			///@param	normal_			[out]三角形の法線、省略不可
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
				const math::t_vector3&	apex1_,
				const math::t_vector3&	apex2_,
				const math::t_vector3&	apex3_,
				const math::t_vector3&	lineP_,
				const math::t_vector3&	lineDir_,
				math::t_vector3*		normal_,
				math::t_vector3*		intersection_ = NULL);

			///@brief	片面三角-線分の衝突位置を求める
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
			bool TriSegCol(
				const math::t_vector3&	apex1_,
				const math::t_vector3&	apex2_,
				const math::t_vector3&	apex3_,
				const math::t_vector3&	segP_,
				const math::t_vector3&	segQ_,
				math::t_vector3*		normal_,
				float*					distance_ = NULL,
				math::t_vector3*		intersection_ = NULL);

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
			bool TriSegColBothFaces(
				const math::t_vector3&	apex1_,
				const math::t_vector3&	apex2_,
				const math::t_vector3&	apex3_,
				const math::t_vector3&	segP_,
				const math::t_vector3&	segQ_,
				math::t_vector3*		normal_,
				float*					distance_ = NULL,
				math::t_vector3*		intersection_ = NULL);

			///@brief	三角ポリゴンと線分の最近接点
			///@param	apexA1_			三角形の頂点1
			///@param	apexA2_			三角形の頂点2
			///@param	apexA3_			三角形の頂点3
			///@param	segP_			線分の始点
			///@param	segQ_			線分の終点
			///@param	proximateL_		[out]線上の最近接点
			///@param	proximateP_		[out]三角形上の最近接点
			///@note	三角形の裏表の定義は左手系に準じる
			void TriSegProximate(
				const math::t_vector3&	apex1_,
				const math::t_vector3&	apex2_,
				const math::t_vector3&	apex3_,
				const math::t_vector3&	segP_,
				const math::t_vector3&	segQ_,
				math::t_vector3*		proximateL_ = NULL,
				math::t_vector3*		proximateP_ = NULL);

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
				const math::t_vector3&	apexA1_,
				const math::t_vector3&	apexA2_,
				const math::t_vector3&	apexA3_,
				const math::t_vector3&	apexB1_,
				const math::t_vector3&	apexB2_,
				const math::t_vector3&	apexB3_);

		#if 0
		// どう言う仕様で作っていたか不明
		// 未完成な感じ

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
			///@param   penetrateA_		[out]三角形A側の進入量、省略可
			///@param   penetrateB_		[out]三角形B側の進入量、省略可
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
			bool TriTriCol(
				const math::t_vector3&	apexA1_,
				const math::t_vector3&	apexA2_,
				const math::t_vector3&	apexA3_,
				const math::t_vector3&	apexB1_,
				const math::t_vector3&	apexB2_,
				const math::t_vector3&	apexB3_,
				math::t_vector3*		intersectionA_ = NULL,
				math::t_vector3*		intersectionB_ = NULL,
				math::t_vector3*		penetrateA_ = NULL,
				math::t_vector3*		penetrateB_ = NULL);

			///@brief	両面三角-両面三角の衝突点と衝突方向を求める
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
			///@param   penetrateA_		[out]三角形A側の進入量、省略可
			///@param   penetrateB_		[out]三角形B側の進入量、省略可
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
			bool TriTriColBoth(
				const math::t_vector3&	apexA1_,
				const math::t_vector3&	apexA2_,
				const math::t_vector3&	apexA3_,
				const math::t_vector3&	apexB1_,
				const math::t_vector3&	apexB2_,
				const math::t_vector3&	apexB3_,
				math::t_vector3*		intersectionA_ = NULL,
				math::t_vector3*		intersectionB_ = NULL,
				math::t_vector3*		penetrateA_ = NULL,
				math::t_vector3*		penetrateB_ = NULL);
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
				const math::t_vector3&	apex1_,
				const math::t_vector3&	apex2_,
				const math::t_vector3&	apex3_,
				const math::t_vector3&	apexD_,
				const math::t_vector3&	segP_,
				const math::t_vector3&	segQ_,
				math::t_vector3*		intersection_ = NULL);

			#if 0
				///@brief	AABB-点の衝突
				///@param	halfSize_	AABBのボックス半径
				///@param	center_		AABBの中心点
				///@param	pt_			点の座標
				bool AabbPtCol(
					const math::t_vector3&	center_,
					const math::t_vector3&	halfSize_,
					const math::t_vector3&	pt_);
			#else
				///@brief	AABB-点の衝突
				///@param	aabb_	AABB
				///@param	pt_		点の座標
				///@retval	true	衝突あり
				///@retval	false	衝突なし
				///@note
				///
				/// 	AABB-点 大きなAABBと点に射影して計算
				/// 	Axis Aligned Bounding Box 点に射影して計算
				///
				bool AabbPtCol(
					const CAabb3&	aabb_,
					const math::t_vector3&		pt_);

			#endif

			///@brief	AABB-点最近接点を取得する
			///@param	aabb_			AABB
			///@param	p_				点の座標
			///@param	proximate_		[out]最近接点
			void AabbPtProximate(
				const CAabb3&     aabb_,
				const math::t_vector3&	p_,
				math::t_vector3*		proximate_);

			///@brief	AABB-球の衝突を判定する
			///@param	aabb_		AABB
			///@param	p_			点の座標
			///@param	radius_		球の半径
			bool AabbSphereCol(
				const CAabb3&     aabb_,
				const math::t_vector3&	p_,
				float					radius_);

			///@brief	AABB-光線の衝突を判定する
			///@param	aabb_		AABB
			///@param	rayP_		光線の原点
			///@param	rayDir_		光線の方向と長さ
			///@param	normal_		[out]衝突面の法線、省略可
			///@retval	衝突点までのrayDir_に対する係数
			float AabbRayCol(
				const CAabb3&			aabb_,
				const math::t_vector3&	rayP_,
				const math::t_vector3&	rayDir_,
				math::t_vector3*		normal_ = NULL);

			///@brief	AABB-平面の衝突を判定する
			///@param	aabb_		AABB
			///@param	planeN_		平面の法線
			///@param	planeD_		平面の法線方向オフセット
			///@retval	<0	ボックスが平面の裏側
			///@retval	>0	ボックスが平面の表側
			///@retval	==0	平面と交差する
			float AabbPlaneCol(
				const CAabb3&			aabb_,
				const math::t_vector3&	planeN_,
				const float				planeD_);

			///@brief	AABB-移動平面の衝突を判定する
			///@param	aabb_		AABB
			///@param	relAabbV_	平面に対するAABBの相対速度
			///@param	planeN_		平面の法線
			///@param	planeD_		平面の法線方向オフセット
			///@retval	<0	ボックスが平面の裏側
			///@retval	>0	ボックスが平面の表側
			///@retval	==0	平面と交差する
			float AabbPlaneCol(
				const CAabb3&			aabb_,
				const math::t_vector3&	relAabbV_,
				const math::t_vector3&	planeN_,
				const float&			planeD_);

			#if 0
				///@brief	AABB-AABB
				///@note	Axis Aligned Bounding Box 大きなAABBと点に射影して計算する
				///@param	cencterA_	AABB_Aの中心点
				///@param	halfSizeA_	AABB_Aのボックス半径
				///@param	centerB_	AABB_Bの中心点
				///@param	halfSizeB_	AABB_Bのボックス半径
				bool AabbAabbCol(
					const math::t_vector3&	cencterA_,
					const math::t_vector3&	halfSizeA_,
					const math::t_vector3&	centerB_,
					const math::t_vector3&	halfSizeB_);
			#else
				///@brief	AABB-AABBの動的衝突点を求める。
				///@note	AABB_A基準にして、AABB_Aは静止しているとして計算する
				///@param	aabbA_		AABB_A
				///@param	aabbB_		AABB_B
				///@param	relAabbBV_	Aに対するBの相対速度
				///@retval	交差開始するrelAabbBV_に対するパラメータ化距離
				bool AabbAabbCol(
					const CAabb3&		aabbA_,
					const CAabb3&		aabbB_);

				///@brief	AABB-AABBの動的衝突点を求める。
				///@note	AABB_A基準にして、AABB_Aは静止しているとして計算する
				///@param	aabbA_		AABB_A
				///@param	aabbB_		AABB_B
				///@param	relAabbBV_	Aに対するBの相対速度
				///@retval	交差開始するrelAabbBV_に対するパラメータ化距離
				float AabbAabbCol(
					const CAabb3&		aabbA_,
					const CAabb3&		aabbB_,
					const math::t_vector3&	relVelocityB_);
			#endif

			#if 0
				///@brief		OBB-OBBが衝突判定においていくつかの処理を省いて
				/// 				衝突する可能性があるかを判定する。<br>
				///@attension	この関数がtrueを返しても、衝突していない場合がある。
				/// 				厳密に判定する場合はObbObbCol()を使用すること。
				///@param	worldA_		OBB_Aに適用されているワールド変換行列
				///@param	halfSizeA_	OBB_Aのボックス半径
				///@param	worldB_		OBB_Bに適用されているワールド変換行列
				///@param	halfSizeB_	OBB_Bのボックス半径
				///@retval	true		衝突の可能性がある
				///@retval	false		衝突なし
				bool ObbObbColPossibility(
					const math::t_matrix4&	worldA_,
					const math::t_vector3&	halfSizeA_,
					const math::t_matrix4&	worldB_);
					const math::t_vector3&	halfSizeB_,

				///@brief	OBB－OBBの衝突を判定する
				///@param	halfSizeA_	OBB_Aのボックス半径
				///@param	worldA_		OBB_Aに適用されているワールド変換行列
				///@param	halfSizeB_	OBB_Bのボックス半径
				///@param	worldB_		OBB_Bに適用されているワールド変換行列
				///@retval	true		衝突あり
				///@retval	false		衝突なし
				bool ObbObbCol(
					const math::t_vector3&	halfSizeA_,
					const math::t_matrix4&	worldA_,
					const math::t_vector3&	halfSizeB_,
					const math::t_matrix4&	worldB_);

				///@brief	凸?と書いてあったが本当か？
				// 本当
				// メッシュとそのローカル上の点
				// 一番近い面への投影とその方向の食い込みを返す
				bool MeshPtCol(
					const CTriMesh&			mesh_,
					const math::t_vector3&	p_,			// 衝突位置（メッシュのローカル）
					float*					depth_,
					math::t_vector3*		n_);


				///@brief	メッシュと線分衝突判定片面
				// 一番近い面への投影とその方向の食い込みを返す
				bool MeshSegCol(
					const CTriMesh&			mesh_,
					const math::t_vector3&	segP_,					// 線分の始点
					const math::t_vector3&	segDir_,				// 線分の方向// と長さ？だったか
					float*					depth_);


				///@brief	凸?と書いてあったが本当か？
				// 本当
				// メッシュ衝突位置
				// 体積を持った衝突図形になってしまうので、
				// 何回か行うことで物体を完全に分離する
				// colPosA_：Aの衝突位置（Aローカル）
				// colPosB_：Bの衝突位置（Bローカル）
				// penetrate_：衝突方向と深さ(絶対座標)
				bool MeshColPos(
					const CTriMesh&			meshA_,
					const CTriMesh&			meshB_,
					const math::t_matrix4&	worldA_,		// それぞれの位置姿勢
					const math::t_matrix4&	worldB_,
					math::t_vector3*		colPosA_,		// Aの衝突位置（Aローカル）
					math::t_vector3*		colPosB_,		// Bの衝突位置（Bローカル）
					math::t_vector3*		penetrate_);	// 衝突方向と深さ(絶対座標)

				///@brief	凸?と書いてあったが本当か？
				/// 本当
				/// メッシュ衝突位置
				/// 体積を持った衝突図形になってしまうので、
				/// 速度を利用してこれから起こるであろう
				/// 衝突の瞬間の衝突瞬間の位置を求める
				bool MeshColPos(
					const CTriMesh&		meshA_,
					const CTriMesh&		meshB_,
					const math::t_matrix4&	worldA_,				// それぞれの位置姿勢
					const math::t_matrix4&	worldB_,
					const math::t_vector3&	vA_,					// 速度
					const math::t_vector3&	vB_,
					const math::t_vector3&	omegaA_,				// 角速度
					const math::t_vector3&	omegaB_,
					math::t_vector3*		colPosA_,				// Aの衝突位置（A側の中心から）
					math::t_vector3*		colPosB_,				// Bの衝突位置（B側の中心から）
					math::t_vector3*		penetrate_);			// 衝突方向と相対速度
			#endif
			// @}


			///@brief	視垂台-球の衝突を判定する
			///@param	result_    結果が入る平面
			///@param	plane_     元になる平面
			///@param	transform_ トランスフォーム行列
			void MakePlaneTransformed(
				CPlane*                 result_,
				const CPlane&			plane_,
				const math::t_matrix4&	transform_);

			///@brief	視垂台-球の衝突を判定する
			///@param	frustum_
			///@param	point_
			///@param	radius_		平面の法線方向オフセット
			///@retval	<0	ボックスが平面の裏側
			///@retval	>0	ボックスが平面の表側
			///@retval	==0	平面と交差する
			bool FrustumSphereCol(
				const CFrustum&			frustum_,
				const math::t_vector3&	point_,
				float				    radius_);

			///@attention これで作った視垂台はサイドの平面（m_sides）以外のメンバーは無効である
			///@brief	視垂台のトランスフォーム
			///@param	result_    結果が入る視垂台
			///@param	frustum_   元になる視垂台
			///@param	transform_ トランスフォーム行列
			void MakeFrustumTransformed(
				CFrustum*               result_,
				const CFrustum&			frustum_,
				const math::t_matrix4&	transform_);


		// }
	}
}
#endif
	