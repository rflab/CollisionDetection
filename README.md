collision
=========
2D/3D衝突判定
mathに依存しているのでインクルードしたほうがいいのかも、作ったのだいぶ前なので殆ど忘れた。
[大体の衝突判定実装済み（ヘッダファイル）](https://github.com/rflab/CollisionDetection/blob/master/collision/collisiondetectfunction.h)
# サポートしているオブジェクト
```
- 2D衝突判定
		- 点(Pt)
		- 円(Circle)
		- 線(line)
		- 線分(seg)
		- 半直線/レイ(Ray)
		- Axis Aligned Bounding Box(Aabb)
- 3D衝突判定
		- 点(Pt)
		- 球(Sphere)
		- 線(line)
		- 線分(seg)
		- 半直線/レイ(Ray)
		- 平面(Plane)
		- 三角形(Tri)
		- 四角形(Square)
		- Axis Aligned Bounding Box(Aabb)
		- Oriented Bounding Box(Obb)
```
