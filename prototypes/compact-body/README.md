# Compact body — source-grounded CAD prototype

This is the compact, low-cost direction selected by Daniel, developed in an isolated branch of his Asimov repository. It uses two SO-101 **follower** arms and an original LeKiwi three-wheel mechanism, a fixed camera head and printable body structure. It is approximately 391 mm high in the reference scene, with a 340 mm deck. The earlier ten images were visual concepts; this deliverable is actual CAD.

**Release stage: CAD prototype. No fabricated hardware, motor operation, continuous trajectory validation or payload qualification has been performed.** Geometry validity passes for the new body; sampled unrestricted motion and conservative power screening reveal limitations. Read the failures below before purchasing or printing a full set.

## 1. Cost and scope

The planning total is **$444.79**, consisting of $375.81 in repository price quotes, $33.98 for two filament spools at an observed manufacturer listing, and **$35 in unquoted allowances**. Shipping, tax, duties and revised power hardware are excluded. A third spool brings the estimate to **$461.78**. Slice all parts before finalizing filament quantity: all-solid source prints plus new body would exceed 3 kg; source arms are normally printed with infill.

| Target | Honest scope |
|---|---|
| $100 | A simple stationary camera/display body or shell; not this two-arm mobile robot. Twelve source-priced arm motors alone are $166.68. |
| $200 | A one-follower stationary build is plausible using the repository's $121.94 follower estimate plus simple body and camera; not modeled or priced as a second complete build here. |
| $500 | This wired two-follower, three-wheel CAD prototype is a plausible parts target using source quotes. Landed cost, matching wheels and qualified power hardware remain unresolved. |

The first version uses an existing PC/Spark host, printer and tools. No onboard Pi, leader teleoperation arm, battery, autonomous navigation computer, moving neck, telescoping waist or three-finger hand is included. The original source grippers remain. Motor cables, horns and servo screws must be confirmed in the purchased motor bundle. The $35 allowances are disclosed design extras, not falsely attributed to the repository.

See [bom.csv](bom.csv) for quantities, source URLs, supplier links, price basis, notes and extended prices; [budget.json](checks/budget.json) for machine-readable arithmetic. The camera is the **$12.98 USB camera listed in LeKiwi's BOM**, not a claimed current Alibaba quote. Its exact housing/lens geometry still needs a purchase drawing or physical sample. Do not substitute a cheap camera on price alone and assume an exact mounting fit.

At fixed non-motor costs, the fifteen motors must average at most $17.79 each to stay within $500 before shipping and tax, or approximately $16.66 with a third spool. This is a price constraint, not evidence that a suitable current offer exists.

## 2. Source revisions and provenance

| Source | Pinned revision | Use |
|---|---|---|
| [Daniel's Asimov repository](https://github.com/egc365/asimov-1/tree/35ae7b3581ce36762bfddb00b8fc5e7c957ee7fa) | `35ae7b3581ce36762bfddb00b8fc5e7c957ee7fa` | Host repository, design context and original humanoid reference. |
| [Daniel's SO-ARM100 fork](https://github.com/egc365/SO-ARM100/tree/5f6d2b876a53a4872e405b991dd925556c9e38a4) | `5f6d2b876a53a4872e405b991dd925556c9e38a4` | Follower BOM, printed mechanisms, original base STEP, source URDF and gauges. |
| [LeKiwi](https://github.com/SIGRobotics-UIUC/LeKiwi/tree/efa608d7ee5a495a4803b1d28cd0c955b4f1e033) | `efa608d7ee5a495a4803b1d28cd0c955b4f1e033` | Three-wheel mechanism, top-plate hole geometry, original assembly guide and wired BOM. |

The full Asimov is a 1.2 m, 35 kg humanoid with industrial actuators and metal structure. This compact body does **not** retain its actuator interfaces or claim to run its control software unchanged. Its README links `mechanical/FABRICATION_MANIFEST.csv`, but that file was absent at the pinned revision; a complete public Asimov purchase BOM was not obtained. The manufacturer BOM form requests contact details and was not submitted.

Original source snapshots are preserved in [source/](source/); [source_hashes.json](checks/source_hashes.json) records SHA256 hashes and sizes for measured CAD/URDF baselines. `SO101_base_assembly_frame.step` is a **derived, rigidly translated reference**, extracted from the original STEP assembly and moved so its underside is Z=0. Its unchanged volume matches the individual source base. The two new base parts change only four mounting bores; no motor geometry or gear dimensions are invented.

Source SO-ARM and LeKiwi license texts are retained. They are Apache-2.0 upstream assets. Existing Asimov hardware/software license files remain authoritative for repository-native work. Preserve upstream attribution and license files when redistributing the retained or modified base; no upstream trademark endorsement is claimed.

## 3. Editable CAD and actual exports

| File | Purpose |
|---|---|
| [design.py](design.py), [parameters.json](parameters.json) | Parameter-driven, editable CadQuery model. All dimensions mm. |
| `cad/compact_body.xbf` in [cad_bundle.zip](cad_bundle.zip) | Native OpenCascade XCAF assembly with named parts. |
| `cad/compact_body.step` in [cad_bundle.zip](cad_bundle.zip) | Neutral CAD assembly of the 19 body parts, importable in FreeCAD and other STEP-capable tools. |
| `cad/` in [cad_bundle.zip](cad_bundle.zip) | Individual STEP parts and print STLs. Purchased standoffs are represented as envelopes, without modeled threads. |
| `cad/robot_reference_assembly.glb`, generated by render.py | Full visual reference: body CAD plus source arm/base URDF meshes. Included in the complete review download, not the smaller GitHub CAD bundle. Source revision differences apply. |
| [assembly_views.png](drawings/assembly_views.png) | Engineering preview from CAD/source meshes; no image generation. |
| [mounting_plan.svg](drawings/mounting_plan.svg) | Dimensioned deck interface plan; use STEP for fabrication. |
| [verify.py](verify.py), [checks/](checks/) | Reproducible checks and actual results. |
| [printed_source_parts.csv](printed_source_parts.csv) | Exact unchanged source print files and quantities. |

CadQuery 2.7.0 / OpenCascade 7.8.1.1 was used as the installed CAD tool. FreeCAD was not installed; these are genuine B-rep solids, not render-only meshes. The editable design source generates both XBF and STEP. The native assembly includes the new body and modified bases; moving arm mechanisms and drive hardware are source references in GLB and their original repositories, not falsely certified native solids.

The model has a solid 8 mm deck, 3 mm column/head walls, 3.6 mm nominal M3 clearance bores and 0.6 mm nominal face-edge clearance. The head is 124 × 84 × 80 mm. Arm bases are mounted at X=±105, Y=−20, Z=78. The camera shelf uses two strap slots rather than an unverified PCB pattern.

## 4. Coordinate system and measured interfaces

Assembly frame: +Z up; camera faces −Y; X spans the two arm bases. Z=0 is the source LeKiwi lower plate's top, **not ground**. The source reference meshes extend to approximately Z=−32.94; the head top is Z=358.

| Interface | Fact, measurement or assumption | Coordinates / fit |
|---|---|---|
| I01: LeKiwi top plate → added deck | Measured from source STL, not physical hardware | Existing hole axes (±40,±40), fitted radius ~1.74253 mm; radial fit residual ~0.0075 mm. New bores 3.6 mm. Plate top Z=50, purchased standoffs 20 mm, deck underside Z=70. |
| I02: source arm base → added deck | New modification to actual source STEP | Four bores per base at local (−35,−20),(35,−20),(−50,20),(50,20); cut diameter 3.6 through lower 20 mm. Pattern is printed in modified bases, not claimed to already exist upstream. |
| I03: deck → column foot | New, modeled interface | Four outer bores (±35,35),(±35,75); four inner bores (±20,45),(±20,65). M3 through fasteners, no printed threads. |
| I04: tube → bridges | New, modeled interface | Two transverse X-axis M3 bores at Y=45 and65, Z=89 and275. 64 mm outer span. |
| I05: head → upper bridge | New, modeled interface | Four vertical M3 bores (±20,45),(±20,65). |
| I06: face → head bosses | New, modeled interface | Four Y-axis M3 bores at X=±56, Z=288 and348. 0.6 mm nominal edge gap. |
| I07: camera shelf | New, provisional fit | 54×40 mm shelf, 24 mm optical aperture, 3×28 mm strap slots; exact camera fit/field-of-view/USB cable exit not established. |
| I08: original drive mechanism | Upstream mechanism retained | Assemble original wheel hubs/mounts/plates to upstream guide. Source URDF mount differs from printable v2 mount; source URDF base is a visual/kinematic reference, not an exact v2 fit certification. |

The SO101 individual source base uses a different vertical axis than its assembly placement; the provided derived base eliminates that ambiguity. Source SO101 URDF base geometry is aligned to this assembly using a +90° Z rotation and bounding-box datums. This is a documented geometric alignment, not a servo calibration.

## 5. What was actually tested

[results.json](checks/results.json) contains the authoritative pass/fail/conditional records. Every saved part was reimported in a separate process from the builder. The native XBF and STEP were reopened, with checks for valid solids, positive volume, component count and dimensional/volume agreement.

| Requirement | Evidence | Status / implication |
|---|---|---|
| R01: budget near $500 | Itemized BOM arithmetic | Conditional; $444.79 source-based estimate, excluding landed charges and unresolved upgrades. |
| R02: editable native and STEP | Native + neutral reopen tests | Pass: 19 valid positive-volume solids; agreement tolerance 0.001 mm bbox, relative volume 1e-6. |
| R03: known plate interface | Four measured axes matched independently | Pass for source geometry; physical plate and standoff fit unmeasured. |
| R04: M3 clearances | Independent 1.79/1.81 mm cylindrical probes against exported deck | Pass: 3.6 mm modeled bores. Assumed −0.2 mm hole error leaves 0.4 mm diametral clearance against 3.0 mm screw. |
| R05: shell fit | Explicit edge tolerance stack | Conditional: 0.6 mm nominal edge gap minus two assumed 0.2 mm errors =0.2 mm. Actual printer error must be measured. |
| R06: body static interference | Every body-part pair tested by exact B-rep common volume | Pass; shared faces/contact are allowed. Source electronics, camera, cables and moving mechanism hardware are outside this check. |
| R07: arm motion | FCL mesh checks at seeded source-URDF poses | Restricted. Full source limits contain body and inter-arm collisions. See every joint state/pair in motion_samples.json; clear sample does not approve a trajectory. |
| R08: input validation | Attempt to build wall=1.0 mm | Pass: rejected with explicit dimension error. |
| R09: source full STEP | Imported original SO101 assembly | Fail: 266 solids, full shape invalid. Named assembly loading also encountered duplicate component names. Original retained; new body solids checked independently. |
| R10: electrical supply | Conservative fifteen-servo current screen | Fail for simultaneous stall with three assumed 5 A supplies; supply ratings/adapter current capacity still need verification. |
| R11: payload | Simple arm and deck arithmetic | Unqualified. 100 g at300 mm plus0.632 kg source arm mass gives shoulder static screen1.224 Nm, ~76% of source6 V stall torque1.618 Nm. At5 V, continuous payload is not known. |
| R12: stability | Source wheel centers, explicit assumed mass/COM | Conditional. Actual wheel contact patches, COM, acceleration/braking/slopes and external loads unmeasured. |
| R13: print exports | Independent STL topology, units, dimension and volume check | All15 print exports are checked for watertightness, consistent winding, Z=0 placement, <0.16 mm dimension error and <1% volume difference. See print_export_checks.json. |

The body material upper bound is approximately **1.307 kg** at assumed solid PLA density1.24 g/cm³, excluding source arm bases and purchased standoffs. It is not a weighed robot mass. The source arm's URDF total mass is0.632006 kg; this is a source model value. Original LeKiwi URDF mass entries were not accepted as measured hardware masses.

Deck screening treats one arm as a cantilever load over65 mm, width87 mm, thickness8 mm, E=1500 MPa. It gives about0.434 MPa nominal bending stress and0.102 mm deflection for arm self-weight. This simplified screen assumes a solid section and excludes creep, bolt preload, stress concentrations and print layer weakness; it is not finite-element analysis or a proof load.

The current screen uses2.5 A from a7.4 V servo variant as a conservative reference, **not a measured5 V current**. Fifteen stalled servos give37.5 A; six in one arm give15 A. A5 A branch does not cover that screen. Supply size, fuses, wire gauge, board trace current and servo overload shutdown must be qualified together. Do not parallel the three supplies or apply12 V to7.4 V servos.

## 6. Failure modes and unresolved interactions

| ID | Failure / cause | Evidence | Required next action |
|---|---|---|---|
| F01 | Arm strikes head/column/deck or other arm | Actual sampled mesh collisions | Implement trajectory collision checking, calibrate joint datums and validate guarded workspace. Do not enable unrestricted source limits. |
| F02 | Supply droop/reset or hot connector under stalled load | Current screen exceeds assumed branch rating | Check exact supply/board data; bench-test one actuator then one bank with current logging, fuse and voltage monitoring. |
| F03 | Stall torque mistaken for continuous payload | Continuous rating absent; high static torque fraction | Measure sustained-duty temperature/current and actual load torque. No100 g working payload rating issued. |
| F04 | Print hole shrink / flange cracking / layer separation | Tolerance stack assumes printer errors; source base modified | Print source servo gauges and one modified base coupon; inspect holes, washer bearing surfaces and bolt access before full assembly. |
| F05 | Robot tips during reach/braking or tether pull | Small wheel-contact triangle; unmeasured actual COM | Weigh assembled parts; measure COM/contact patch; test slow, tether-managed movement with restraints before autonomy. |
| F06 | Camera will not fit or aperture clips image | Housing/lens/field-of-view dimensions unresolved | Obtain exact source-listed camera drawing or sample, adjust shelf/aperture, verify full frame. |
| F07 | Wheels/hub substitute incompatible or unavailable | Legacy VEX purchase URL failed; alternative wheel differs | Purchase exact matched source wheel/hub set or measure/re-CAD hub; do not assume100 mm is equivalent to4 inches. |
| F08 | Source CAD revisions disagree | SO101 full STEP invalid; LeKiwi mount reference differs | Use source print files/manual for mechanisms; reconcile current vendor hardware against pinned files. |
| F09 | Control integration missing | Compact servos differ from Asimov actuator architecture | Use source LeRobot SO101/LeKiwi interfaces as integration starting points; dual-follower mapping and full body controller have not been implemented. |
| F10 | Cables foul joints or rail remains powered after disconnect | Cable geometry and board backfeed unknown | Route strain relief and service loops; test physical DC motor-rail disconnect under guarded bench conditions. |

An Asimov-inspired safety approach here means visible limits, collision checking and a physical motor-power disconnect. It is not a claim that software implements fictional laws or guarantees safety.

| Coupled factors | Low / high or alternative cases | Actual coverage |
|---|---|---|
| Source joint angles × body position | Neutral;48 seeded full-range states;12 single-joint±5° states | Actual sampled geometry checks; results recorded. Both arms use the same sampled joint vector. Independent combinations and continuous paths NOT RUN. |
| Hole error × screw size | Nominal3.6; −0.2 mm assumed error;3.0 mm screw maximum | Analytical stack + exported bore probes; physical print NOT RUN. |
| Face fit × edge error |0.6 nominal;0.2 mm assumed error on each edge | Analytical stack; physical assembly NOT RUN. |
| Payload × reach × supply voltage |100 g /300 mm /5 V build;6 V stall reference | Analytical screen only; continuous thermal/load test NOT RUN. |
| Infill × COM × braking |Source infill vs solid shell upper bound; assumed1 kg base; actual contact triangle | Analytical assumptions; instrumented tipping/braking test NOT RUN. |
| Camera variant × aperture × cable exit |Source inexpensive USB camera vs32 mm PCB option | Exact housing/drawing missing; camera fit and field-of-view NOT RUN. |

## 7. Build and inspection sequence

1. Review BOM unresolved items F02/F06/F07 before placing a full order. Confirm **two follower** sets (12 C001 arm servos), three compatible base servos, servo cables/horns/fasteners and exact wheel hub fit. Obtain actual supply ratings and motor-board current limits.
2. Fetch pinned sources with `python fetch_sources.py`; this refuses to modify an existing checkout at a different revision. Read the upstream arm instructions and [LeKiwi assembly guide](https://github.com/SIGRobotics-UIUC/LeKiwi/blob/efa608d7ee5a495a4803b1d28cd0c955b4f1e033/Assembly.md).
3. Print the source `STL/Gauges/Gauge_0.STL` and `Gauge_tight_1.STL`. Measure printer hole/edge errors. Re-enter measured clearances in `parameters.json`, regenerate CAD and rerun checks before printing every part.
4. Print one modified SO101 base first. Its four new bores are already in the STL. Check that M3 screws pass, washer/head seats are supported and accessible, and no original servo housing/cable path is affected. Original source base STEP is preserved for comparison.
5. Print new body STLs and unchanged mechanism files listed in `printed_source_parts.csv`. Use upstream settings for original parts. Deck structural arithmetic assumes a solid8 mm section: print the deck solid or recalculate for the chosen infill. Suggested prototype walls are at least5 perimeters; this is a starting assumption, not a strength qualification. Orient the column with its open rear upward when possible. Confirm supports and bed fit in the slicer. A340 mm deck leaves only5 mm each side on a350 mm bed, including any brim.
6. Assemble the three original LeKiwi wheel modules and original two plates according to the pinned guide. Keep the original source electronics mount. The CAD preview's legacy URDF frame is a reference; it does not override source print/assembly interfaces.
7. Install four **20 mm M3 female/female standoffs** in the source top plate's(±40,±40) holes, usingM3x12 screws and washers through the7 mm plate. Fasten the added8 mm deck from above withM3x12. Check thread engagement against the purchased standoff drawing; modeled standoffs have plain bore envelopes.
8. Bolt each modified arm base to the deck with fourM3x35 through bolts, washers and nuts. Base flanges lie at localZ2.5…17.5 around the new bore locations; check physical washer bearing and engagement. Follow upstream follower-arm assembly for all remaining arm parts, retaining original grippers. Fasteners supplied with servos are needed in addition to the body mounting list.
9. Fasten column foot to deck at its four outer holes withM3x20 through bolts/nuts/washers. At the four inner holes, connect lower bridge/foot/deck withM3x30 through bolts. Place tube over bridges and use twoM3x75 transverse bolts per bridge through the64 mm span. Nuts and washers remain accessible. Fit the upper bridge before the head. Retain rear cover with two hook-and-loop straps; leave a service opening until wiring is checked.
10. Fasten head bottom to upper bridge with fourM3x12 bolts/washers/nuts. Use four modeled5 mm camera spacers andM3x16 bolts for the shelf. Strap the exact camera in place only after checking lens alignment. Fit optional amber LEDs with retained holders/adhesive appropriate to the purchased LEDs; the CAD has apertures, not unverified snap fits. Attach face to its bosses with fourM3x16 Y-axis bolts/washers/nuts.
11. Bench-configure/calibrate one servo bank at a time using upstream tooling. Assign IDs per bank; separate USB interfaces may reuse IDs across banks. Connect three external source-compatible rails through individually rated fuses/disconnects. KeepUSB logic and motor-power paths explicit, do not parallel DC rails, and measure for backfeed. Exact fuse/wire/switch SKUs await current qualification. No motor command was sent in this work.
12. Start with an unpowered fit inspection. Then perform guarded low-speed bench tests with no payload, checking actual rail voltage/current, cable motion and power disconnect. Every trajectory must be checked against actual calibrated geometry. The sampled neutral or±5° checks do not certify a safe commissioning sequence. Measure COM and test restrained base movement before evaluating payload or autonomous movement.

Added body fastener schedule:8×M3x35 arm bases;4×M3x20 outer foot;4×M3x30 lower bridge stack;4×M3x75 transverse bridges;4×M3x12 head;4×M3x16 face;4×M3x16 camera;8×M3x12 standoff ends;4×20 mm M3 standoffs; washers/nuts as above;2 rear-cover straps and1 camera strap. Sizes are CAD-derived starting selections. Verify actual purchased head/nut dimensions and thread engagement. The$10 added-hardware allowance is not a supplier quote or guarantee these are all in the assortment.

## 8. Reproduce and review

From this folder inside an Asimov checkout. The complete review ZIP contains `compact-body/`; extract it into `<asimov-checkout>/prototypes/`. That review ZIP already has unpacked CAD, so skip the `zipfile -e` line there. When using the GitHub branch, extract `cad_bundle.zip` as shown:

```sh
python -m pip install -r requirements.txt
python -m zipfile -e cad_bundle.zip .
python fetch_sources.py
python design.py
python verify.py
python check_print_exports.py
python render.py
```

The builder loads `parameters.json`, generates native/neutral assemblies and centered, Z=0 print STLs. STEP parts remain in assembly coordinates. `verify.py` runs in a separate process. Mesh collision checks use the exact source visual meshes and record the states; the preview PNG uses simplified continuous surfaces, while the reference GLB retains source triangles. Fresh-process B-rep reopen does not repair or silently validate the invalid upstream arm STEP. A previous actual source-assembly result may be reused only when its SHA256 still matches.

The original arm-base tessellation produced13 zero-area triangles per modified base that confused STL topology checking. The builder removes only those zero-area triangles, without filling holes or welding vertices; it logs maximum removed triangle area and volume change in `checks/stl_export_cleanup.json`. Independent export checks then assess the saved mesh. This does not repair or validate the upstream full-arm STEP.

The highest supportable conclusion is: **an inspectable, editable compact body CAD prototype and source-derived budget/BOM are delivered; a physically functional, fully toleranced, power-qualified robot has not yet been demonstrated.** Purchase and fabrication readiness depend on the concrete unresolved interfaces and tests above.
