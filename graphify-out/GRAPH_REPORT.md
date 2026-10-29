# Graph Report - SmartGrid  (2026-10-29)

## Corpus Check
- 100 files · ~84,892 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 4 file(s) not represented in the graph (top: (none) 2, .ico 1, .bat 1)

## Summary
- 1279 nodes · 3105 edges · 85 communities (48 shown, 37 thin omitted)
- Extraction: 90% EXTRACTED · 10% INFERRED · 0% AMBIGUOUS · INFERRED: 319 edges (avg confidence: 0.9)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- OLP Export Pipeline
- SmartRibbon UI Controls
- Mesh Flow Export Gate
- OEM Reference & Standards
- Mesh Quality Metrics
- Face Grid / BBox Generation
- 3D Mesh Viewer
- Main Window View Logic
- Curve Conditioner & Resampling
- Mesh Flow Worker & Selection
- Main Window Core
- Adaptive Grid Generator
- Normal & TCP Frame Computation
- Region & Lane Planning
- BBox Route Generator
- Surface Safety Utilities
- Process State Planner
- Reference Metrics
- Reference Comparison
- OLP Reference Parsers
- Module Cluster 20
- Module Cluster 21
- Module Cluster 22
- Module Cluster 23
- Module Cluster 25
- Module Cluster 26
- Module Cluster 27
- Module Cluster 28
- Module Cluster 29
- Module Cluster 30
- Module Cluster 31
- Module Cluster 32
- Module Cluster 33
- Module Cluster 34
- Module Cluster 35
- Module Cluster 36
- Module Cluster 37
- Module Cluster 38
- Module Cluster 39
- Module Cluster 40
- Module Cluster 41
- Module Cluster 42
- Module Cluster 43
- Module Cluster 44
- Module Cluster 45
- Module Cluster 46
- Module Cluster 47
- Module Cluster 49
- Module Cluster 50
- Module Cluster 52
- Module Cluster 53
- Module Cluster 54
- Module Cluster 55
- Module Cluster 56
- Module Cluster 57
- Module Cluster 58
- Module Cluster 59
- Module Cluster 60
- Module Cluster 61
- Module Cluster 62
- Module Cluster 63
- Module Cluster 64
- Module Cluster 65
- Module Cluster 66
- Module Cluster 73
- Module Cluster 74
- Module Cluster 76
- Module Cluster 77
- Module Cluster 78
- Module Cluster 79
- Module Cluster 80
- Module Cluster 81
- Module Cluster 83
- Module Cluster 84

## God Nodes (most connected - your core abstractions)
1. `MainWindow` - 80 edges
2. `PaintRoute` - 59 edges
3. `SmartRibbon` - 58 edges
4. `PaintPass` - 44 edges
5. `MeshViewer` - 39 edges
6. `preprocess()` - 35 edges
7. `generate_mesh_flow_route()` - 35 edges
8. `build_regions()` - 24 edges
9. `resample_arc()` - 24 edges
10. `GenerationParams` - 23 edges

## Surprising Connections (you probably didn't know these)
- `R5 Alternating (Serpentine) Direction Design Rule` --semantically_similar_to--> `Boustrophedon / Serpentine Path Planning (Visual Concept)`  [INFERRED] [semantically similar]
  convertmdfiles/SmartGrid_Mesh_Flow_Specification_v4_Bundle/SmartGrid_Mesh_Flow_Engineering_Specification_v4.md → app/ui/icons/icon.png
- `Tool Center Point (TCP) and Standoff Terminology` --semantically_similar_to--> `Robot Spray Gun / Tool End Effector`  [INFERRED] [semantically similar]
  convertmdfiles/WHERE_WE_LEFT_OFF_AND_PENDING_WORK.md → app/ui/icons/icon.png
- `MotionSegmentKind` --uses--> `test_connectors_are_never_painted()`  [INFERRED]
  app/path/path_model.py → tests/test_transition_planner.py
- `Export Mesh (.stl toolpath tube export)` --semantically_similar_to--> `Export Mesh Mode Plan Summary`  [INFERRED] [semantically similar]
  README.md → .claude/PRPs/plans/export-mesh-mode.plan.md
- `Trimesh Triangle-Plane Intersection` --shares_data_with--> `app/export/mesh_export.py module (planned)`  [INFERRED]
  README.md → .claude/PRPs/plans/export-mesh-mode.plan.md

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **Four Path Generation Modes** — readme_boundary_box, readme_adaptive, readme_conform, readme_mesh_flow [EXTRACTED 0.90]
- **Four Export Formats** — readme_export_json, readme_export_csv, readme_export_olp, readme_export_mesh [EXTRACTED 0.90]
- **Export Mesh Feature Implementation Set** — export_mesh_mode_plan_mesh_export_module, export_mesh_mode_plan_ribbon_wiring, export_mesh_mode_plan_main_window_slot, export_mesh_mode_plan_test_fixture [EXTRACTED 0.85]

## Communities (85 total, 37 thin omitted)

### Community 0 - "OLP Export Pipeline"
Cohesion: 0.06
Nodes (39): _check_traversal_direction(), compare_to_reference(), ComparisonReport, CorrespondenceConfidence, LaneComparison, _pass_centroid(), split_contiguous_gun_on_runs(), split_contiguous_gun_on_runs_including_unknown() (+31 more)

### Community 1 - "SmartRibbon UI Controls"
Cohesion: 0.06
Nodes (31): export_route_csv(), _write_metadata(), _corner_cnt(), export_abb_rapid(), export_cad_dxf(), export_delmia_apt(), export_durr(), export_fanuc_ls() (+23 more)

### Community 2 - "Mesh Flow Export Gate"
Cohesion: 0.08
Nodes (33): enforce_export_gate(), ExportBlocked, ExportKind, AcceptanceTolerances, ProcessConstraints, ToolAxisConvention, _status_from_findings(), validate_route_hard_gates() (+25 more)

### Community 3 - "OEM Reference & Standards"
Cohesion: 0.05
Nodes (49): SmartGrid App Icon (Robot Arm, Boustrophedon Path, Grid), Boustrophedon / Serpentine Path Planning (Visual Concept), Robot Spray Gun / Tool End Effector, ABB External Axes — Not Computable (positioner/turntable), ABB Export Fixes (PROC per region, runJob, wobj from filename), OLP Export Standard Review Document, Dürr Export Fixes (SETOBJECT, SETBRUSH per region, REGION comment), Dürr WAIT_PAINTPOSITION — Not Computable (conveyor layout) (+41 more)

### Community 4 - "Mesh Quality Metrics"
Cohesion: 0.08
Nodes (12): _conn_to_dict(), export_route_json(), _NumpyEncoder, _pass_to_dict(), _route_to_dict(), RobotManagerDialog, RobotProfileDialog, get_active_profile() (+4 more)

### Community 5 - "Face Grid / BBox Generation"
Cohesion: 0.08
Nodes (4): _group_for_face_grid(), _group_for_mesh(), _on_adaptive_done(), run()

### Community 6 - "3D Mesh Viewer"
Cohesion: 0.10
Nodes (20): DirectionalDeviation, edge_length_cv(), _face_samples(), _measurement_scale_mm(), neighbor_normal_variation(), _pick_shared_subdivision_depth(), _sample_mesh_faces(), _SampleBatch (+12 more)

### Community 7 - "Main Window View Logic"
Cohesion: 0.12
Nodes (20): _MeshFlowWorker, generate_mesh_flow_route(), resolve_lane_pitch(), LanePitchStrategy, build_regions(), RegionConfidence, flat_panel_mesh(), test_build_regions_accepted_for_clean_flat_region() (+12 more)

### Community 9 - "Mesh Flow Worker & Selection"
Cohesion: 0.10
Nodes (13): _FakeRibbon, _FakeViewer, _FakeWorker, _make_fake_window(), qapp(), _reset_fake_worker(), test_dropped_regions_are_warned_about_and_routes_still_forwarded(), test_excluding_every_region_blocks_generation() (+5 more)

### Community 10 - "Main Window Core"
Cohesion: 0.11
Nodes (15): resample_and_smooth(), approximate_curvature(), curvature_change_penalty(), second_differences(), segment_directions(), smoothness_penalty(), test_curvature_change_penalty_f52(), test_resample_and_smooth_keeps_endpoints_fixed() (+7 more)

### Community 11 - "Adaptive Grid Generator"
Cohesion: 0.12
Nodes (13): apply_process_states(), plan_transitions(), connect_passes(), reorder_passes_nearest_neighbor(), Connection, MeshQualityReport, MotionSegmentKind, PaintPass (+5 more)

### Community 12 - "Normal & TCP Frame Computation"
Cohesion: 0.09
Nodes (6): SmartRibbon, qapp(), test_default_path_target_is_unaffected_by_mesh_flow_flag(), test_get_path_target_returns_mesh_flow_when_checked(), test_mesh_flow_radio_visibility_follows_flag_disabled(), test_mesh_flow_radio_visibility_follows_flag_enabled()

### Community 13 - "Region & Lane Planning"
Cohesion: 0.15
Nodes (14): run(), _axes(), _batch_normals_tangents(), _clip_inside(), _compute_surface_basis(), _filter_chains(), generate_adaptive_grid_route(), _resample_anchored() (+6 more)

### Community 14 - "BBox Route Generator"
Cohesion: 0.15
Nodes (11): preprocess(), _to_pyvista(), classify_regions(), test_conform_normals_and_tangents_stored(), test_generate_returns_paint_route(), test_standoff_within_tolerance_on_cylinder(), _capture(), test_mesh_surface_matches_baseline() (+3 more)

### Community 15 - "Surface Safety Utilities"
Cohesion: 0.14
Nodes (11): closest_point_distance(), connected_components(), inside_mesh_mask(), outward_tcp_offset(), segment_penetrates(), test_inside_mesh_mask_detects_interior_point_on_watertight_mesh(), test_inside_mesh_mask_none_on_non_watertight(), test_segment_penetrates_catches_a_chord_through_a_thin_feature() (+3 more)

### Community 16 - "Process State Planner"
Cohesion: 0.14
Nodes (3): _build_orientation_report(), Mesh Flow Open Decision 6: Canonical Identifier Backward Compat, Mesh Flow Phase 0 Rename Audit

### Community 17 - "Reference Metrics"
Cohesion: 0.14
Nodes (5): MeshData, _classify_regions_scored(), classify_regions_with_confidence(), _region_vectors(), MeshModel

### Community 18 - "Reference Comparison"
Cohesion: 0.13
Nodes (6): _offset_route_by_standoff(), build_tcp_frames(), interpolate_normals(), _query_for(), test_tcp_frame_orthonormal(), test_interpolate_normals_restricted_to_region_ignores_a_closer_decoy_part()

### Community 19 - "OLP Reference Parsers"
Cohesion: 0.18
Nodes (10): display_label_for(), RenamePolicy, resolve_path_mode_for_load(), OpenDecisionBlocked, test_default_policy_is_label_only(), test_display_label_for_is_identity_until_the_rename_is_actually_applied(), test_dual_write_policy_is_passthrough(), test_migrate_policy_round_trips_correctly() (+2 more)

### Community 20 - "Module Cluster 20"
Cohesion: 0.16
Nodes (5): _find_seed_vertices(), generate_geodesic_route(), _isocontour(), _per_point_normals(), rdp_simplify()

### Community 22 - "Module Cluster 22"
Cohesion: 0.13
Nodes (5): export_route_mesh(), _tube(), sample_route(), test_export_route_mesh_raises_on_empty_routes(), test_export_route_mesh_writes_stl()

### Community 23 - "Module Cluster 23"
Cohesion: 0.13
Nodes (7): _flip_route(), merge_routes(), check_orientation(), detect_collisions(), detect_overlaps(), PaintRoute, sample_route()

### Community 25 - "Module Cluster 25"
Cohesion: 0.21
Nodes (6): check_collision(), measure_boundary_gap(), measure_coverage(), measure_standoff_deviation(), measure_surface_pitch(), validate_route()

### Community 26 - "Module Cluster 26"
Cohesion: 0.31
Nodes (12): check_straightness(), check_tool_flip(), _make_pass(), test_check_collision_watertight_inside(), test_check_collision_watertight_outside(), test_check_straightness_passes_for_straight_line(), test_check_straightness_raises_for_curved_path(), test_check_straightness_skips_short_pass() (+4 more)

### Community 27 - "Module Cluster 27"
Cohesion: 0.15
Nodes (3): _make_plane_grid(), _region_face_normal(), _render_plane_quad()

### Community 28 - "Module Cluster 28"
Cohesion: 0.16
Nodes (4): RegionBoundaryPolicy, tool_axis_convention_for(), _significant_connected_components(), test_connectors_are_never_painted()

### Community 29 - "Module Cluster 29"
Cohesion: 0.21
Nodes (6): compute_slice_config(), compute_slice_planes(), _outward_sign(), slice_region(), test_slice_config_top_region(), test_slice_returns_none_when_plane_misses()

### Community 30 - "Module Cluster 30"
Cohesion: 0.23
Nodes (9): max_orientation_jump(), orientation_continuity(), pointing_axis(), segment_lengths(), total_length(), test_f36_f37_segment_and_total_length(), test_f44_f45_orientation_continuity(), test_pointing_axis_abb_quaternion_round_trip() (+1 more)

### Community 31 - "Module Cluster 31"
Cohesion: 0.13
Nodes (4): _CollisionWorker, _LoadWorker, _PathWorker, _ViewerImportWorker

### Community 35 - "Module Cluster 35"
Cohesion: 0.15
Nodes (15): main Branch (v1.5.3 Baseline), Phase 1: Project Genesis & Early MVP, Phase 2: UI Ribbon & 4 Path Modes, Decision: Canonical Axis Filtering for Curved Geometries, Decision: Even-Pitch Formula, Decision: Jig/Holder 5-Face Continuous Painting, Rejected: Hemisphere Filter for Segment Slicing, ABB RAPID Exporter Task (.mod) (+7 more)

### Community 36 - "Module Cluster 36"
Cohesion: 0.15
Nodes (15): Rejected: Per-Waypoint Normal Offsetting on Curved Faces, Decision: Uniform mean_n Standoff in Conform & BBox, Open Question: Failing Unit Tests in test_generator.py, Open Question: Scalability on 500MB Production Assemblies, Boundary Box Path Mode, Boundary Box Path Mode, Boustrophedon Toolpath, Choset 2000 Boustrophedon Cellular Decomposition (+7 more)

### Community 37 - "Module Cluster 37"
Cohesion: 0.20
Nodes (7): intersect_region(), stitch_segments(), test_closed_loop(), test_floating_point_tolerance(), test_single_segment_returns_one_chain(), test_three_collinear_segments_form_one_chain(), test_two_disconnected_chains()

### Community 38 - "Module Cluster 38"
Cohesion: 0.26
Nodes (5): load_mesh(), _load_step(), _box_with_flipped_faces(), test_load_mesh_leaves_already_consistent_mesh_unchanged(), test_load_mesh_repairs_inconsistent_stl_winding()

### Community 39 - "Module Cluster 39"
Cohesion: 0.33
Nodes (5): estimate_primary_direction(), local_scan_frame(), _pca_principal_axis(), _region_mean_normal(), MeshFlowRegion

### Community 40 - "Module Cluster 40"
Cohesion: 0.20
Nodes (3): _centroid_to_region(), on_click(), _release()

### Community 41 - "Module Cluster 41"
Cohesion: 0.27
Nodes (4): generate_bbox_route(), _make_passes(), prune_collinear(), resample_arc()

### Community 43 - "Module Cluster 43"
Cohesion: 0.33
Nodes (5): _arc_length(), _build_planes_combined(), _dominant_dir(), _filter_polylines(), generate_route()

### Community 44 - "Module Cluster 44"
Cohesion: 0.31
Nodes (5): _danger_btn(), _large_btn(), _make_icon(), _primary_btn(), _small_btn()

### Community 45 - "Module Cluster 45"
Cohesion: 0.24
Nodes (11): Phase 5: Mesh Flow Coverage Fixes, core_generator.py (Mesh Flow), lane_planner.py (Mesh Flow), Mesh Flow Beta Path Mode, region_manager.py (Mesh Flow), safety_validator.py (Mesh Flow), v1.9 Curvature-Adaptive Lane Pitch, v1.9 Cover All Regions Toggle (+3 more)

### Community 46 - "Module Cluster 46"
Cohesion: 0.29
Nodes (3): _arc_length(), _region_components(), _significant_polylines()

### Community 47 - "Module Cluster 47"
Cohesion: 0.22
Nodes (10): EXPORTER_FUNCTION_SIGNATURE pattern (csv_export.py mirror), main_window._export_mesh slot, app/export/mesh_export.py module (planned), Rationale: scope exclusions (NOT Building), Ribbon Export Mesh button/signal wiring, Insight: shapely already a transitive dependency via trimesh[easy], Export Mesh Mode Plan Summary, Insight: trimesh.creation.sweep_polygon as tube primitive (+2 more)

### Community 49 - "Module Cluster 49"
Cohesion: 0.22
Nodes (3): _make_bbox_face(), _make_grid_lines(), _make_multiline()

### Community 50 - "Module Cluster 50"
Cohesion: 0.36
Nodes (3): cylinder_mesh(), flat_plane_mesh(), unit_box_mesh()

### Community 54 - "Module Cluster 54"
Cohesion: 0.33
Nodes (7): feature/smartgrid-noether-boundary-spray Branch (Frozen), Phase 3: Noether Port & Audit, ROS-Industrial Noether Reference Repo, Adaptive Path Mode, Mesh Flow (beta) Path Mode, Region Face Classifier, Per-waypoint TCP Frame (Noether)

### Community 56 - "Module Cluster 56"
Cohesion: 0.33
Nodes (6): Phase 4: OLP Review & Reconstruction, Decision: Auto-Direction Selection & Radio Lock, Rejected: Geodesic Iso-Contours for Mesh Surface, Decision: Slice+Stitch Replaces Geodesic Iso-Contours, OLP Industry Review (Stellantis/Jackson E/Vikram), generator.py (Mesh Surface)

### Community 58 - "Module Cluster 58"
Cohesion: 0.40
Nodes (5): Conform Path Mode, Mesh Surface (golden baseline, removed from UI), Rationale: keep Mesh Surface as regression baseline, Ramer / Douglas-Peucker Simplification, Trimesh Triangle-Plane Intersection

### Community 62 - "Module Cluster 62"
Cohesion: 0.67
Nodes (3): Rejected: _clip_inside in Adaptive Grid, Decision: Removal of _clip_inside from Adaptive Grid, Face Grid Adaptive Path Mode

### Community 63 - "Module Cluster 63"
Cohesion: 0.67
Nodes (3): Canonical Axis Fix for Named Face Regions, face_grid_generator.py (Tilted Mean_n Root Cause), Yellow Plane vs Red Plane Canonical Rule

## Knowledge Gaps
- **61 isolated node(s):** `OLP Export (RoboDK/VC/DELMIA/Gcode)`, `Decision: Trimesh BVH Off-by-One Tie-Breaker`, `NumPy Numerical Computing`, `Potpourri3d Geodesic Distance (Mesh Surface Mode)`, `PySide6 Qt GUI Framework` (+56 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 494 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **37 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `MainWindow` connect `Curve Conditioner & Resampling` to `SmartRibbon UI Controls`, `Mesh Flow Export Gate`, `Mesh Quality Metrics`, `Face Grid / BBox Generation`, `Mesh Flow Worker & Selection`, `Normal & TCP Frame Computation`, `Process State Planner`, `Reference Metrics`, `OLP Reference Parsers`, `Module Cluster 22`, `Module Cluster 23`, `Module Cluster 24`, `Module Cluster 31`, `Module Cluster 32`, `Module Cluster 33`, `Module Cluster 34`, `Module Cluster 42`, `Module Cluster 48`, `Module Cluster 51`, `Module Cluster 57`?**
  _High betweenness centrality (0.128) - this node is a cross-community bridge._
- **Are the 18 inferred relationships involving `MainWindow` (e.g. with `OpenDecisionBlocked` and `ToolAxisConvention`) actually correct?**
  _`MainWindow` has 18 INFERRED edges - model-reasoned connections that need verification._
- **What connects `OLP Export (RoboDK/VC/DELMIA/Gcode)`, `Decision: Trimesh BVH Off-by-One Tie-Breaker`, `NumPy Numerical Computing` to the rest of the system?**
  _61 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `OLP Export Pipeline` be split into smaller, more focused modules?**
  _Cohesion score 0.05546218487394958 - nodes in this community are weakly interconnected._
- **Why does `SmartRibbon` connect `Normal & TCP Frame Computation` to `Module Cluster 32`, `Face Grid / BBox Generation`, `Curve Conditioner & Resampling`, `Module Cluster 44`, `Process State Planner`, `Module Cluster 51`, `Module Cluster 21`, `Module Cluster 24`, `Module Cluster 61`?**
  _High betweenness centrality (0.099) - this node is a cross-community bridge._
- **Are the 21 inferred relationships involving `PaintRoute` (e.g. with `export_route_csv()` and `export_route_json()`) actually correct?**
  _`PaintRoute` has 21 INFERRED edges - model-reasoned connections that need verification._
- **Should `SmartRibbon UI Controls` be split into smaller, more focused modules?**
  _Cohesion score 0.05649122807017544 - nodes in this community are weakly interconnected._