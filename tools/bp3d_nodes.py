"""BoneSim node table: GLB node name -> BodyParts3D FMA ids, segment, group, names.

Imported by tools/build_assets.py.  Side suffix _L / _R (subject's left/right).
"""

SIDES = {"R": ("right", "右", "Right"), "L": ("left", "左", "Left")}

# (R ids, L ids) for bilateral single-bone nodes
UPPER = [
    # node, segment, nameJa, nameEn, (R, L)
    ("clavicle", "clavicle", "鎖骨", "clavicle", ("FMA13322", "FMA13323")),
    ("scapula", "scapula", "肩甲骨", "scapula", ("FMA13395", "FMA13396")),
    ("humerus", "humerus", "上腕骨", "humerus", ("FMA23130", "FMA23131")),
    ("ulna", "ulna", "尺骨", "ulna", ("FMA23467", "FMA23468")),
    ("radius", "radius", "橈骨", "radius", ("FMA23464", "FMA23465")),
    ("scaphoid", "carpalsProximal", "舟状骨", "scaphoid", ("FMA24435", "FMA24436")),
    ("lunate", "carpalsProximal", "月状骨", "lunate", ("FMA24437", "FMA24438")),
    ("triquetrum", "carpalsProximal", "三角骨", "triquetrum", ("FMA24439", "FMA24440")),
    ("pisiform", "carpalsProximal", "豆状骨", "pisiform", ("FMA24441", "FMA24442")),
    ("trapezium", "hand", "大菱形骨", "trapezium", ("FMA24443", "FMA24444")),
    ("trapezoid", "hand", "小菱形骨", "trapezoid", ("FMA23725", "FMA24445")),
    ("capitate", "hand", "有頭骨", "capitate", ("FMA24446", "FMA24447")),
    ("hamate", "hand", "有鉤骨", "hamate", ("FMA24448", "FMA24449")),
    ("metacarpal1", "thumbMeta", "第1中手骨", "first metacarpal", ("FMA24464", "FMA24465")),
    ("metacarpal2", "hand", "第2中手骨", "second metacarpal", ("FMA24466", "FMA24467")),
    ("metacarpal3", "hand", "第3中手骨", "third metacarpal", ("FMA24468", "FMA24469")),
    ("metacarpal4", "hand", "第4中手骨", "fourth metacarpal", ("FMA24470", "FMA24471")),
    ("metacarpal5", "hand", "第5中手骨", "fifth metacarpal", ("FMA24472", "FMA24473")),
    ("thumbProx", "thumbProx", "母指基節骨", "proximal phalanx of thumb", ("FMA24450", "FMA65470")),
    ("thumbDist", "thumbDist", "母指末節骨", "distal phalanx of thumb", ("FMA24459", "FMA23951")),
    ("indexProx", "indexProx", "示指基節骨", "proximal phalanx of index finger", ("FMA24451", "FMA71915")),
    ("indexMid", "indexMid", "示指中節骨", "middle phalanx of index finger", ("FMA24455", "FMA23938")),
    ("indexDist", "indexDist", "示指末節骨", "distal phalanx of index finger", ("FMA24460", "FMA23953")),
    ("middleProx", "middleProx", "中指基節骨", "proximal phalanx of middle finger", ("FMA24452", "FMA71908")),
    ("middleMid", "middleMid", "中指中節骨", "middle phalanx of middle finger", ("FMA24456", "FMA23940")),
    ("middleDist", "middleDist", "中指末節骨", "distal phalanx of middle finger", ("FMA24461", "FMA23955")),
    ("ringProx", "ringProx", "環指基節骨", "proximal phalanx of ring finger", ("FMA24453", "FMA71916")),
    ("ringMid", "ringMid", "環指中節骨", "middle phalanx of ring finger", ("FMA24457", "FMA23942")),
    ("ringDist", "ringDist", "環指末節骨", "distal phalanx of ring finger", ("FMA24462", "FMA23957")),
    ("littleProx", "littleProx", "小指基節骨", "proximal phalanx of little finger", ("FMA24454", "FMA66791")),
    ("littleMid", "littleMid", "小指中節骨", "middle phalanx of little finger", ("FMA24458", "FMA23944")),
    ("littleDist", "littleDist", "小指末節骨", "distal phalanx of little finger", ("FMA24463", "FMA23959")),
]

LOWER = [
    ("femur", "femur", "大腿骨", "femur", ("FMA24474", "FMA24475")),
    ("patella", "patella", "膝蓋骨", "patella", ("FMA24486", "FMA24487")),
    ("tibia", "tibia", "脛骨", "tibia", ("FMA24477", "FMA24478")),
    ("fibula", "tibia", "腓骨", "fibula", ("FMA24480", "FMA24481")),
    ("talus", "talus", "距骨", "talus", ("FMA24482", "FMA24483")),
    ("calcaneus", "foot", "踵骨", "calcaneus", ("FMA24497", "FMA24498")),
    ("navicular", "foot", "舟状骨（足）", "navicular", ("FMA24500", "FMA24501")),
    ("cuboid", "foot", "立方骨", "cuboid", ("FMA24528", "FMA24529")),
    ("cuneiformMedial", "foot", "内側楔状骨", "medial cuneiform", ("FMA24521", "FMA24522")),
    ("cuneiformIntermediate", "foot", "中間楔状骨", "intermediate cuneiform", ("FMA24523", "FMA24524")),
    ("cuneiformLateral", "foot", "外側楔状骨", "lateral cuneiform", ("FMA24525", "FMA24526")),
    ("metatarsal1", "foot", "第1中足骨", "first metatarsal", ("FMA24507", "FMA24508")),
    ("metatarsal2", "foot", "第2中足骨", "second metatarsal", ("FMA24509", "FMA24510")),
    ("metatarsal3", "foot", "第3中足骨", "third metatarsal", ("FMA24511", "FMA24512")),
    ("metatarsal4", "foot", "第4中足骨", "fourth metatarsal", ("FMA24513", "FMA24514")),
    ("metatarsal5", "foot", "第5中足骨", "fifth metatarsal", ("FMA24515", "FMA24516")),
]
# toe phalanges + sesamoid merged per foot (R ids, L ids)
TOES = (
    ["FMA43253", "FMA32650", "FMA32634", "FMA32642", "FMA32652", "FMA32636", "FMA32644", "FMA32654",
     "FMA32638", "FMA32646", "FMA32656", "FMA32640", "FMA230986", "FMA32658", "FMA45097"],
    ["FMA43254", "FMA32651", "FMA32635", "FMA32643", "FMA32653", "FMA32637", "FMA32645", "FMA32655",
     "FMA32639", "FMA32647", "FMA32657", "FMA32641", "FMA230988", "FMA32659", "FMA45098"],
)
RIBS = {  # rib number -> (R, L)
    1: ("FMA7857", "FMA7987"), 2: ("FMA7882", "FMA8012"), 3: ("FMA7909", "FMA8039"),
    4: ("FMA7957", "FMA8148"), 5: ("FMA8066", "FMA8093"), 6: ("FMA8175", "FMA8202"),
    7: ("FMA8229", "FMA8256"), 8: ("FMA8283", "FMA8310"), 9: ("FMA8364", "FMA8391"),
    10: ("FMA8445", "FMA8472"), 11: ("FMA8531", "FMA8532"), 12: ("FMA8533", "FMA8534"),
}
COSTAL_CARTILAGE = (
    ["FMA7875", "FMA7886", "FMA7913", "FMA7976", "FMA8070", "FMA8194", "FMA8248", "BP28"],
    ["FMA8005", "FMA8031", "FMA8058", "FMA8167", "FMA8112", "FMA8221", "FMA8275", "BP24"],
)
CERVICAL = ["FMA12519", "FMA12520", "FMA12521", "FMA12522", "FMA12523", "FMA12524", "FMA12525"]
THORACIC = ["FMA9165", "FMA9187", "FMA9209", "FMA9248", "FMA9922", "FMA9945", "FMA9968",
            "FMA9991", "FMA10014", "FMA10037", "FMA10059", "FMA10081"]
LUMBAR = ["FMA13072", "FMA13073", "FMA13074", "FMA13075", "FMA13076"]
# intervertebral disc *below* the named vertebra (not exported, used for joint centres)
DISCS = {
    "C2": "FMA25058", "C3": "FMA13896", "C4": "FMA13897", "C5": "FMA13898", "C6": "FMA13899",
    "C7": "FMA13900", "T1": "FMA10458", "T2": "FMA13495", "T3": "FMA13500", "T4": "FMA13501",
    "T5": "FMA13502", "T6": "FMA13503", "T7": "FMA13504", "T8": "FMA13505", "T9": "FMA13506",
    "T10": "FMA13507", "T11": "FMA13508", "T12": "FMA13509", "L1": "FMA16033", "L2": "FMA16034",
    "L3": "FMA16035", "L4": "FMA16036", "L5": "FMA16037",
}
SKULL = ["FMA52734", "FMA52735", "FMA52788", "FMA52789", "FMA52738", "FMA52739", "FMA52736",
         "FMA52740", "FMA53649", "FMA53650", "FMA52892", "FMA52893", "FMA53647", "FMA53648",
         "FMA53645", "FMA53646", "FMA53655", "FMA53656", "FMA54737", "FMA54738", "FMA9710",
         # upper teeth
         "FMA55682", "FMA55699", "FMA55690", "FMA55683", "FMA55700", "FMA55691", "FMA55799",
         "FMA55681", "FMA55698", "FMA55689", "FMA55680", "FMA55697", "FMA55688", "FMA55798"]
MANDIBLE = ["FMA52748",
            "FMA57143", "FMA55704", "FMA55693", "FMA57141", "FMA55703", "FMA55692", "FMA55687",
            "FMA57142", "FMA55705", "FMA55694", "FMA57140", "FMA55706", "FMA55695", "FMA55686"]
NASAL_CARTILAGE = "FMA71704"   # landmark only (nose tip)
OCCIPITAL = "FMA52735"


def node_table():
    """Return ordered dict: node -> dict(segment, group, nameJa, nameEn, fma(list))."""
    T = {}

    def add(node, segment, group, ja, en, ids):
        T[node] = dict(segment=segment, group=group, nameJa=ja, nameEn=en,
                       fma=list(ids) if isinstance(ids, (list, tuple)) else [ids])

    add("skull", "skull", "skull", "頭蓋骨（上顎歯を含む）", "Cranium and facial bones (with upper teeth)", SKULL)
    add("mandible", "skull", "skull", "下顎骨（下顎歯を含む）", "Mandible (with lower teeth)", MANDIBLE)
    add("hyoid", "skull", "skull", "舌骨", "Hyoid bone", "FMA52749")
    cj = ["環椎（第1頸椎）", "軸椎（第2頸椎）"] + [f"第{i}頸椎" for i in range(3, 8)]
    for i, f in enumerate(CERVICAL):
        n = f"C{i+1}"
        add(n, n, "spine", cj[i], ["Atlas (C1)", "Axis (C2)"][i] if i < 2 else f"{['third','fourth','fifth','sixth','seventh'][i-2]} cervical vertebra (C{i+1})", f)
    for i, f in enumerate(THORACIC):
        add(f"T{i+1}", "thorax", "thorax", f"第{i+1}胸椎", f"Thoracic vertebra T{i+1}", f)
    add("manubrium", "thorax", "thorax", "胸骨柄", "Manubrium of sternum", "FMA7486")
    add("sternumBody", "thorax", "thorax", "胸骨体", "Body of sternum", "FMA7487")
    add("xiphoid", "thorax", "thorax", "剣状突起", "Xiphoid process", "FMA7488")
    for k, (sR, ja, en) in (("R", ("R", "右", "Right")), ("L", ("L", "左", "Left"))):
        idx = 0 if k == "R" else 1
        for r, ids in RIBS.items():
            add(f"rib{r}_{k}", "thorax", "thorax", f"第{r}肋骨（{ja}）", f"{en} rib {r}", ids[idx])
        add(f"costalCartilage_{k}", "thorax", "thorax", f"肋軟骨（{ja}）", f"{en} costal cartilages 1-10", COSTAL_CARTILAGE[idx])
    for i, f in enumerate(LUMBAR):
        add(f"L{i+1}", f"L{i+1}", "spine", f"第{i+1}腰椎", f"Lumbar vertebra L{i+1}", f)
    add("sacrum", "pelvis", "pelvis", "仙骨", "Sacrum", "FMA16202")
    add("hipBone_R", "pelvis", "pelvis", "寛骨（右）", "Right hip bone", "FMA16586")
    add("hipBone_L", "pelvis", "pelvis", "寛骨（左）", "Left hip bone", "FMA16587")
    for k in ("R", "L"):
        idx = 0 if k == "R" else 1
        _, ja, en = SIDES[k]
        for node, seg, nja, nen, ids in UPPER:
            grp = "hand" if seg not in ("clavicle", "scapula", "humerus", "ulna", "radius") else "upperLimb"
            add(f"{node}_{k}", f"{seg}_{k}", grp, f"{nja}（{ja}）", f"{en} {nen}", ids[idx])
        for node, seg, nja, nen, ids in LOWER:
            grp = "foot" if seg in ("talus", "foot") else "lowerLimb"
            add(f"{node}_{k}", f"{seg}_{k}", grp, f"{nja}（{ja}）", f"{en} {nen}", ids[idx])
        add(f"toePhalanges_{k}", f"foot_{k}", "foot", f"足の趾骨・種子骨（{ja}）",
            f"{en} phalanges of toes and sesamoids", TOES[idx])
    return T
