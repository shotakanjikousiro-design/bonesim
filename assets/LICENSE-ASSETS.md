# Asset licence / 素材のライセンス

`assets/skeleton.glb` and the geometry-derived data in `assets/rig.json`
(joint centres, axes, landmarks, dims) are derived from **BodyParts3D**:

> **BodyParts3D, © The Database Center for Life Science (DBCLS), licensed under
> CC Attribution-Share Alike 2.1 Japan (CC BY-SA 2.1 JP).**
> BodyParts3D, Copyright© 2008 ライフサイエンス統合データベースセンター licensed by CC表示-継承2.1 日本
>
> - Licence: https://creativecommons.org/licenses/by-sa/2.1/jp/deed.en
> - Original licence page: http://lifesciencedb.jp/bp3d/info_en/license/index.html
> - Paper: Mitsuhashi N, Fujieda K, Tamura T, Kawamoto S, Takagi T, Okubo K.
>   *BodyParts3D: 3D structure database for anatomical concepts.* Nucleic Acids Res. 2009;37(Database issue):D782-5.
>   https://doi.org/10.1093/nar/gkn613
> - Data archive DOI: http://doi.org/10.18908/lsdba.nbdc00837-000

Source files used: BodyParts3D release 3.0 (2011-09-15, polygon reduction 95 %), as STL files
redistributed at https://github.com/Kevin-Mattheus-Moerman/BodyParts3D (data under CC BY-SA 2.1 JP;
that repository's code is MIT).

## Modifications (this is a derivative work)
`skeleton.glb` is a **modified** version of the original BodyParts3D meshes:
- re-oriented and scaled into the BoneSim frame (metres; +X subject-left, +Y up, +Z anterior):
  `p = (x, z, −y) / 1000`;
- polygon-decimated (quadric simplification) and smooth vertex normals recomputed;
- grouped / merged into BoneSim nodes (e.g. cranium + facial bones + upper teeth = `skull`,
  costal cartilages 1–10 per side, toe phalanges per foot);
- the skin surface (FMA7163) was decimated and its inner shells removed.

`rig.json` joint centres, axes and landmarks were computed from the BodyParts3D geometry
by `tools/build_assets.py`.

## Licence of the derivative
In accordance with the ShareAlike term, **`assets/skeleton.glb` and the geometric data in
`assets/rig.json` are distributed under the same licence, CC BY-SA 2.1 JP**, with the attribution
above. Any redistribution or further modification must keep this attribution and licence.

これらのデータは教育目的の可視化用です。医療診断・治療の判断には使用しないでください。
(For education and visualisation only; not for clinical decision making.)
