# #################### #
# Model Panel
# #################### #

import bpy
import bmesh
import os
import math
import mathutils
import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom

from mathutils import Vector


# #################### #
# Functions
# #################### #

def safe_float(val):
    """Safely convert a string to float, handling regional comma decimals."""
    if not val or val == "Null": 
        return 0.0
    return float(str(val).replace(',', '.'))

def _get_vgroups(obj):
    if obj and obj.type == 'MESH' and obj.vertex_groups:
        return [(vg.name, vg.name, "") for vg in obj.vertex_groups]
    return [("None", "None", "No vertex groups available")]

def get_general_vertex_groups(self, context):
    settings = context.scene.gymnast_tool_model_props
    if settings.model_type_export in {'HEAD_GEAR', 'MODEL', 'BODY_GEAR', 'RANGED'}:
        return _get_vgroups(settings.selected_object)
    return [("None", "None", "Not applicable")]

def get_weapon1_vertex_groups(self, context):
    if context.scene.gymnast_tool_model_props.model_type_export == 'WEAPON':
        return _get_vgroups(context.scene.gymnast_tool_model_props.weapon_object_1)
    return [("None", "None", "Not applicable")]
    
def get_weapon2_vertex_groups(self, context):
    if context.scene.gymnast_tool_model_props.model_type_export == 'WEAPON':
        return _get_vgroups(context.scene.gymnast_tool_model_props.weapon_object_2)
    return [("None", "None", "Not applicable")]

def get_foot1_vertex_groups(self, context):
    if context.scene.gymnast_tool_model_props.model_type_export == 'FOOT_GEAR':
        return _get_vgroups(context.scene.gymnast_tool_model_props.foot_object_1)
    return [("None", "None", "Not applicable")]

def get_foot2_vertex_groups(self, context):
    if context.scene.gymnast_tool_model_props.model_type_export == 'FOOT_GEAR':
        return _get_vgroups(context.scene.gymnast_tool_model_props.foot_object_2)
    return [("None", "None", "Not applicable")]

def refresh_enum(self, context):
    for area in context.screen.areas:
        if area.type == 'VIEW_3D':
            area.tag_redraw()

def get_triangulated_data(obj):
    """triangulate mesh and return vertices, edges, polygons."""
    mesh = obj.to_mesh()
    bm = bmesh.new()
    bm.from_mesh(mesh)
    bmesh.ops.triangulate(bm, faces=bm.faces[:])
    bm.to_mesh(mesh)
    bm.free()
    return mesh, mesh.vertices, mesh.edges, mesh.polygons

def tetrahedron_volume(p1, p2, p3, p4):
    u, v, w = p2 - p1, p3 - p1, p4 - p1
    return (u.x * (v.y * w.z - v.z * w.y) - u.y * (v.x * w.z - v.z * w.x) + u.z * (v.x * w.y - v.y * w.x)) / 6.0

def calculate_normalized_lcc(macro_pos, p1, p2, p3, p4):
    V = tetrahedron_volume(p1, p2, p3, p4)
    if V == 0: return [0.25, 0.25, 0.25, 0.25]
    lcc = [
        tetrahedron_volume(macro_pos, p2, p3, p4) / V,
        tetrahedron_volume(p1, macro_pos, p3, p4) / V,
        tetrahedron_volume(p1, p2, macro_pos, p4) / V,
        tetrahedron_volume(p1, p2, p3, macro_pos) / V
    ]
    total = sum(lcc)
    return [v / total for v in lcc] if total != 0 else [0.25] * 4

def get_cloth_indices(obj, group_name):
    indices = set()
    if not group_name or group_name == "None": return indices
    vg = obj.vertex_groups.get(group_name)
    if vg:
        for v in obj.data.vertices:
            if any(g.group == vg.index for g in v.groups):
                indices.add(v.index)
    return indices

def get_child_nodes_dict(names, report):
    positions = {}
    missing = []
    for name in names:
        obj = bpy.data.objects.get(name)
        if obj: positions[name] = obj.matrix_world.translation
        else: missing.append(name)
    if missing:
        report({'ERROR'}, f"Missing required child nodes: {', '.join(missing)}")
        return None
    return positions

def get_body_gear_targets(z, p4_mid_z, p4_low_z, profiles, top_key, mid_key, low_key):
    if z >= p4_mid_z:
        return profiles[top_key]
    elif p4_low_z <= z < p4_mid_z:
        return profiles[mid_key]
    else:
        return profiles[low_key]


# ----------------- Transform -----------------

def translate_origin_to_target(obj, target_loc):
    delta = target_loc - obj.location
    if obj.type == 'MESH':
        mesh = obj.data
        for vert in mesh.vertices: vert.co -= delta
        mesh.update()
    obj.location += delta

def align_object_to_basis(obj, origin_loc, target_z, target_y):
    z_dir = (target_z.location - origin_loc).normalized()
    y_dir = (target_y.location - origin_loc).normalized()
    x_dir = y_dir.cross(z_dir).normalized()
    y_dir = z_dir.cross(x_dir).normalized()

    rot_matrix = mathutils.Matrix((x_dir, y_dir, z_dir)).transposed()
    new_quat = rot_matrix.to_quaternion()
    old_quat = obj.rotation_quaternion.copy()
    delta_quat = new_quat @ old_quat.inverted()

    obj.rotation_mode = 'QUATERNION'
    obj.rotation_quaternion = new_quat

    if obj.type == 'MESH' and obj.data:
        for vert in obj.data.vertices: vert.co.rotate(delta_quat.inverted())
        obj.data.update()

def setup_tracking_constraints(obj, loc_target, target_z, target_y, target_x=None, track_x_axis='TRACK_X', use_offset=False):
    obj.constraints.clear()
    copy_loc = obj.constraints.new(type='COPY_LOCATION')
    copy_loc.target = loc_target
    copy_loc.use_offset = use_offset
    
    dz = obj.constraints.new(type='DAMPED_TRACK')
    dz.target = target_z; dz.track_axis = 'TRACK_Z'

    dy = obj.constraints.new(type='DAMPED_TRACK')
    dy.target = target_y; dy.track_axis = 'TRACK_Y'

    if target_x:
        dx = obj.constraints.new(type='DAMPED_TRACK')
        dx.target = target_x; dx.track_axis = track_x_axis

# ----------------- XML Export -----------------

def write_macronode(element, name, pos, mass, is_fixed, p_nodes, child_names):
    lcc = calculate_normalized_lcc(pos, *p_nodes)
    ET.SubElement(element, name, Type="MacroNode",
        X=str(pos.x), Y=str(pos.z), Z=str(-pos.y), Mass=str(mass), Fixed="1" if is_fixed else "0",
        Visible="1", NodesCount="4",
        ChildNode1=child_names[0], ChildNode2=child_names[1], ChildNode3=child_names[2], ChildNode4=child_names[3],
        LCC1=str(lcc[0]), LCC2=str(lcc[1]), LCC3=str(lcc[2]), LCC4=str(lcc[3])
    )

def write_clothnode(element, name, pos, mass, attenuation):
    ET.SubElement(element, name, Type="Node",
        X=str(pos.x), Y=str(pos.z), Z=str(-pos.y), Mass=str(mass), Fixed="0", PinFixed="0",
        Visible="1", Collisible="0", Passive="0", Cloth="1", Attenuation=f"{attenuation:.2f}", Rank="0"
    )

def process_object_nodes(obj, vertices, nodes_element, start_node, prefix, settings, cloth_indices, p_nodes, child_names, macro_indices=None, custom_p_nodes=None, custom_child_names=None):
    if macro_indices is None: macro_indices = set()
    for i, vertex in enumerate(vertices, start=start_node):
        node_name = f"{prefix}Node{i}"
        pos = obj.matrix_world @ vertex.co
        if vertex.index in cloth_indices:
            write_clothnode(nodes_element, node_name, pos, settings.model_export_cloth_mass, settings.model_export_cloth_attenuation)
        elif vertex.index in macro_indices and custom_p_nodes and custom_child_names:
            write_macronode(nodes_element, node_name, pos, settings.model_node_mass, settings.model_node_fixed, custom_p_nodes, custom_child_names)
        else:
            write_macronode(nodes_element, node_name, pos, settings.model_node_mass, settings.model_node_fixed, p_nodes, child_names)
    return start_node + len(vertices)

def store_edge(context, edges, vertices, model_type, edges_element, starting_edge, starting_node, node_name_map=None):
    settings = context.scene.gymnast_tool_model_props
    prefix = settings.model_string_name
    for i, edge in enumerate(edges, start=starting_edge):
        v1, v2 = edge.vertices
        node1, node2 = None, None
        
        if model_type in {"HEAD_GEAR", "WEAPON", "BODY_GEAR", "FOOT_GEAR", "RANGED"}:
            node1 = f"{prefix}Node{v1 + starting_node}"
            node2 = f"{prefix}Node{v2 + starting_node}"
        elif model_type == "MODEL":
            node1 = node_name_map.get(v1) if settings.model_use_pivot else f"{prefix}Node{v1 + starting_node}"
            node2 = node_name_map.get(v2) if settings.model_use_pivot else f"{prefix}Node{v2 + starting_node}"
            
        if not node1 or not node2: continue
        
        length = math.dist(vertices[v1].co, vertices[v2].co)
        ET.SubElement(edges_element, f"{prefix}Edge{i}", Type="Edge", Length=str(length), WithSign="0", Fixed="0", Visible="1",
                      Collisible="1" if settings.model_edge_collisible else "0", SubNodesCount="0", End1=node1, End2=node2)

def store_face(context, faces, vertices, model_type, figures_element, starting_tri, starting_node, node_name_map=None):
    settings = context.scene.gymnast_tool_model_props
    prefix = settings.model_string_name
    for i, face in enumerate(faces, start=starting_tri):
        if len(face.vertices) == 3:
            v1, v2, v3 = face.vertices
            n1, n2, n3 = None, None, None
            
            if model_type in {"HEAD_GEAR", "WEAPON", "BODY_GEAR", "FOOT_GEAR", "RANGED"}:
                n1, n2, n3 = f"{prefix}Node{v1+starting_node}", f"{prefix}Node{v2+starting_node}", f"{prefix}Node{v3+starting_node}"
            elif model_type == "MODEL":
                if settings.model_use_pivot:
                    n1, n2, n3 = node_name_map.get(v1), node_name_map.get(v2), node_name_map.get(v3)
                else:
                    n1, n2, n3 = f"{prefix}Node{v1+starting_node}", f"{prefix}Node{v2+starting_node}", f"{prefix}Node{v3+starting_node}"
            
            if not n1 or not n2 or not n3: continue
            ET.SubElement(figures_element, f"{prefix}Triangle-{i}", Type="Triangle", Node1=n1, Node2=n2, Node3=n3)

def store_edge_attack(context, edges, vertices, edges_element, starting_edge, starting_node, is_first, is_ranged=False):
    settings = context.scene.gymnast_tool_model_props
    prefix = settings.model_string_name
    for i, edge in enumerate(edges, start=1):
        v1, v2 = edge.vertices
        n1, n2 = f"{prefix}Node{v1 + starting_node}", f"{prefix}Node{v2 + starting_node}"
        length = math.dist(vertices[v1].co, vertices[v2].co)
        
        name = f"{prefix}AttackEdge{i}" if is_ranged else (f"{prefix}AttackEdge{i}_1" if is_first else f"{prefix}AttackEdge{i}_2")
        ET.SubElement(edges_element, name, Type="Edge", Length=str(length), WithSign="0", Fixed="0", Visible="1",
                      Collisible="1" if settings.model_edge_collisible else "0", SubNodesCount="0", End1=n1, End2=n2)


# #################### #
# Operators
# #################### #


class ConvertXMLOperator(bpy.types.Operator):
    bl_idname = "model.convert_xml"
    bl_label = "Convert XML to OBJ"
    bl_description = "Convert the Model XML into a Blender object"
    bl_options = {'REGISTER', 'UNDO'}
    
    def execute(self, context):
        props = context.scene.gymnast_tool_model_props
        dependencies_path = bpy.path.abspath(context.scene.gymnast_dependencies_xml) if context.scene.gymnast_dependencies_xml else None
        model_path = bpy.path.abspath(context.scene.gymnast_normal_xml) if context.scene.gymnast_normal_xml else None
        
        if not model_path or not os.path.exists(model_path):
            self.report({'ERROR'}, "Model XML path is missing or invalid.")
            return {'CANCELLED'}
            
        if props.model_use_dependencies and dependencies_path and not os.path.exists(dependencies_path):
            self.report({'ERROR'}, "Dependencies XML path is missing or invalid.")
            return {'CANCELLED'}
        
        # Setup Collections
        model_collection = bpy.data.collections.get("Model") or bpy.data.collections.new("Model")
        if model_collection.name not in context.scene.collection.children: 
            context.scene.collection.children.link(model_collection)

        model_name = os.path.basename(model_path).split('.')[0]
        child_collection = bpy.data.collections.get(model_name) or bpy.data.collections.new(model_name)
        if child_collection.name not in model_collection.children: 
            model_collection.children.link(child_collection)

        temp_obj_path = os.path.join(bpy.path.abspath("//"), f"{model_name}.obj")

        # Parse XMLs Once
        tree = ET.parse(model_path)
        root = tree.getroot()
        
        dep_nodes_dict = {}
        if props.model_use_dependencies and dependencies_path:
            dep_tree = ET.parse(dependencies_path)
            for d_node in dep_tree.getroot().find('Nodes') or []:
                dep_nodes_dict[d_node.tag] = d_node

        nodes = {} # Stores calculated (x, y, z)
        node_index_map = {}
        vertex_counter = 1

        try:
            with open(temp_obj_path, 'w') as obj_file:
                obj_file.write("# Temporary OBJ file generated by Blender Addon\n")
                nodes_section = root.find('Nodes')
                
                if nodes_section is not None:
                    macro_nodes = []
                    
                    # First Pass: Nodes and Centerofmass
                    for node in nodes_section:
                        ntype = node.get('Type')
                        if ntype in ['Node', 'CenterOfMass'] or (ntype == 'MacroNode' and not props.calculate_macronode):
                            x, y, z = safe_float(node.get('X')), safe_float(node.get('Y')), safe_float(node.get('Z'))
                            nodes[node.tag] = (x, y, z)
                            node_index_map[node.tag] = vertex_counter
                            obj_file.write(f"v {x} {-z} {y}\n")
                            vertex_counter += 1
                        elif ntype == 'MacroNode' and props.calculate_macronode:
                            macro_nodes.append(node)
                            
                    # Second Pass: MacroNodes Calculation
                    if props.calculate_macronode:
                        for node in macro_nodes:
                            lcc_pos = [0.0, 0.0, 0.0]
                            for i in range(1, 5):
                                child_name = node.get(f'ChildNode{i}')
                                lcc_val = node.get(f'LCC{i}')
                                
                                if not child_name or child_name == "Null" or not lcc_val: continue
                                lcc = safe_float(lcc_val)

                                if child_name in nodes:
                                    cx, cy, cz = nodes[child_name]
                                elif child_name in dep_nodes_dict:
                                    d_node = dep_nodes_dict[child_name]
                                    cx, cy, cz = safe_float(d_node.get('X')), safe_float(d_node.get('Y')), safe_float(d_node.get('Z'))
                                else:
                                    b_obj = bpy.data.objects.get(child_name)
                                    if b_obj:
                                        cx, cz, cy = b_obj.location
                                        cy = -cy
                                    else: continue
                                    
                                lcc_pos[0] += cx * lcc; lcc_pos[1] += cy * lcc; lcc_pos[2] += cz * lcc
                                
                            nodes[node.tag] = tuple(lcc_pos)
                            node_index_map[node.tag] = vertex_counter
                            obj_file.write(f"v {lcc_pos[0]} {-lcc_pos[2]} {lcc_pos[1]}\n")
                            vertex_counter += 1

                # Write Figures/Faces
                figures_section = root.find('Figures')
                if figures_section is not None:
                    obj_file.write("\n# Faces\n")
                    for figure in figures_section:
                        if figure.get('Type') == 'Triangle':
                            n1, n2, n3 = figure.get('Node1'), figure.get('Node2'), figure.get('Node3')
                            
                            # Dependencies triangle fallback
                            if props.model_use_dependencies:
                                for n in (n1, n2, n3):
                                    if n not in nodes and n in dep_nodes_dict:
                                        d_node = dep_nodes_dict[n]
                                        x, y, z = safe_float(d_node.get('X')), safe_float(d_node.get('Y')), safe_float(d_node.get('Z'))
                                        nodes[n] = (x, y, z)
                                        node_index_map[n] = vertex_counter
                                        obj_file.write(f"v {x} {-z} {y}\n")
                                        vertex_counter += 1

                            if n1 in nodes and n2 in nodes and n3 in nodes:
                                obj_file.write(f"f {node_index_map[n1]} {node_index_map[n2]} {node_index_map[n3]}\n")

            # Import the OBJ file into Blender
            bpy.ops.wm.obj_import(filepath=temp_obj_path)
            imported_obj = context.selected_objects[0]
            imported_obj.name = f"OBJ_{model_name}"
            imported_obj.rotation_euler = (0, 0, 0)
            bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)

            # Move to Triangle collection
            triangle_col = bpy.data.collections.get(f"Triangle_{model_name}") or bpy.data.collections.new(f"Triangle_{model_name}")
            if triangle_col.name not in child_collection.children:
                child_collection.children.link(triangle_col)
                
            for col in imported_obj.users_collection: col.objects.unlink(imported_obj)
            triangle_col.objects.link(imported_obj)
            
            # --- Vertex Groups ---
            if props.add_vertex_group:
                mesh = imported_obj.data
                world_matrix = imported_obj.matrix_world
                
                # Create spatial lookup dictionary
                vert_lut = {(round((world_matrix @ v.co).x, 4), round((world_matrix @ v.co).y, 4), round((world_matrix @ v.co).z, 4)): v.index for v in mesh.vertices}
                
                # Macro Node Rules
                macro_rules = [{"names": set(name.strip() for name in item.names.split(",") if name.strip()), "group": item.group} for item in context.scene.macro_rules]
                
                if macro_rules and nodes_section is not None:
                    for node in nodes_section:
                        if node.get('Type') != 'MacroNode': continue
                        
                        child_nodes = [node.get(f"ChildNode{i}") for i in range(1, 5) if node.get(f"ChildNode{i}") and node.get(f"ChildNode{i}") != "Null"]
                        if not child_nodes: continue
                        
                        child_set = set(child_nodes)
                        for rule in macro_rules:
                            if rule["names"].issubset(child_set):
                                vg_name = rule["group"]
                                vg = imported_obj.vertex_groups.get(vg_name) or imported_obj.vertex_groups.new(name=vg_name)

                                # Fetch the target position directly from dictionary
                                if node.tag in nodes:
                                    nx, ny, nz = nodes[node.tag]
                                    target_pos = Vector((nx, -nz, ny))
                                    key = (round(target_pos.x, 4), round(target_pos.y, 4), round(target_pos.z, 4))
                                    
                                    # Fast dict lookup
                                    if key in vert_lut:
                                        vg.add([vert_lut[key]], 1.0, 'REPLACE')
                                    else: # Safe fallback
                                        for v in mesh.vertices:
                                            if ((world_matrix @ v.co) - target_pos).length < 0.01:
                                                vg.add([v.index], 1.0, 'REPLACE')
                                                break

                # Cloth Nodes
                if props.add_vertex_group_include_cloth and nodes_section is not None:
                    cloth_verts = []
                    for node in nodes_section:
                        if node.get('Type') == 'Node' and node.get('Cloth') == '1' and node.tag in nodes:
                            nx, ny, nz = nodes[node.tag]
                            target_pos = Vector((nx, -nz, ny))
                            key = (round(target_pos.x, 4), round(target_pos.y, 4), round(target_pos.z, 4))
                            
                            if key in vert_lut:
                                cloth_verts.append(vert_lut[key])
                            else:
                                for v in mesh.vertices:
                                    if ((world_matrix @ v.co) - target_pos).length < 0.01:
                                        cloth_verts.append(v.index)
                                        break
                                        
                    if cloth_verts:
                        vgroup = imported_obj.vertex_groups.get("Cloth") or imported_obj.vertex_groups.new(name="Cloth")
                        vgroup.add(cloth_verts, 1.0, 'REPLACE')

        finally:
            # cleanup the temp OBJ file even if it crashes
            if os.path.exists(temp_obj_path):
                os.remove(temp_obj_path)

        self.report({'INFO'}, "Model conversion completed")
        return {'FINISHED'}

class ExportModelToXML(bpy.types.Operator):
    bl_idname = "model.export_to_xml"
    bl_label = "Export OBJ to XML"
    bl_description = "Convert the selected Blender object into an XML file"
    bl_options = {'REGISTER', 'UNDO'}
    filename_ext = ".xml"
    filepath: bpy.props.StringProperty(subtype="FILE_PATH")

    def execute(self, context):
        settings = context.scene.gymnast_tool_model_props
        m_type = settings.model_type_export
        obj = settings.selected_object
        start_node, start_edge, start_tri = settings.model_node_offset, settings.model_edge_offset, settings.model_tri_offset
        prefix = settings.model_string_name
        
        root = ET.Element("Scene")
        nodes_elem, edges_elem, figs_elem = ET.SubElement(root, "Nodes"), ET.SubElement(root, "Edges"), ET.SubElement(root, "Figures")
        
        def process_mesh_export(o, expected_type, child_reqs, cloth_grp_name, is_weapon_or_foot=False, is_attack=False, is_ranged=False):
            nonlocal start_node, start_edge, start_tri
            if not o or o.type != 'MESH': return False
            mesh, verts, edges, faces = get_triangulated_data(o)
            cloth_idx = get_cloth_indices(o, cloth_grp_name) if settings.model_export_cloth else set()
            
            # Custom ChildNodes Override
            macro_idx = set()
            custom_p_nodes = ()
            custom_child_names = []
            if settings.model_custom_childnode:
                custom_objs = [settings.childnode_1_object, settings.childnode_2_object, settings.childnode_3_object, settings.childnode_4_object]
                if all(custom_objs):
                    custom_child_names = [obj.name for obj in custom_objs]
                    custom_p_nodes = tuple(obj.matrix_world.translation for obj in custom_objs)
                    
                    if expected_type == "WEAPON":
                        if o == settings.weapon_object_1:
                            macro_idx = get_cloth_indices(o, settings.macronode_vertex_group_weapon_1)
                        elif o == settings.weapon_object_2:
                            macro_idx = get_cloth_indices(o, settings.macronode_vertex_group_weapon_2)
                    else:
                        macro_idx = get_cloth_indices(o, settings.macronode_vertex_group)
                else:
                    self.report({'WARNING'}, "Custom ChildNodes enabled but missing object references. Defaulting to standard behavior.")
            
            p_dict = get_child_nodes_dict(child_reqs, self.report) if child_reqs else {}
            if p_dict is None and child_reqs: return False
            p_nodes = tuple(p_dict[n] for n in child_reqs) if p_dict else ()

            if is_attack:
                store_edge_attack(context, edges, verts, edges_elem, start_edge, start_node, is_first=True, is_ranged=is_ranged)
                start_node = process_object_nodes(o, verts, nodes_elem, start_node, prefix, settings, set(), p_nodes, child_reqs, macro_idx, custom_p_nodes, custom_child_names)
            elif expected_type == "BODY_GEAR":
                p_up = tuple(get_child_nodes_dict(["NChestS_1", "NChestF", "NChestS_2", "NNeck"], self.report).values())
                p_mid = tuple(get_child_nodes_dict(["NStomachS_1", "NStomachF", "NStomachS_2", "NChest"], self.report).values())
                p_low = tuple(get_child_nodes_dict(["NPelvisF", "NHip_1", "NHip_2", "NStomach"], self.report).values())
                
                profs = {
                    'CHEST': (p_up, ["NChestS_1", "NChestF", "NChestS_2", "NNeck"]),
                    'STOMACH': (p_mid, ["NStomachS_1", "NStomachF", "NStomachS_2", "NChest"]),
                    'HIP': (p_low, ["NPelvisF", "NHip_1", "NHip_2", "NStomach"])
                }
                
                store_edge(context, edges, verts, expected_type, edges_elem, start_edge, start_node)
                store_face(context, faces, verts, expected_type, figs_elem, start_tri, start_node)
                for i, v in enumerate(verts, start=start_node):
                    pos = o.matrix_world @ v.co
                    if v.index in cloth_idx:
                        write_clothnode(nodes_elem, f"{prefix}Node{i}", pos, settings.model_export_cloth_mass, settings.model_export_cloth_attenuation)
                    elif v.index in macro_idx and custom_p_nodes and custom_child_names:
                        write_macronode(nodes_elem, f"{prefix}Node{i}", pos, settings.model_node_mass, settings.model_node_fixed, custom_p_nodes, custom_child_names)
                    else:
                        p_n, c_names = get_body_gear_targets(pos.z, p_mid[3].z, p_low[3].z, profs, settings.model_body_top, settings.model_body_middle, settings.model_body_bottom)
                        write_macronode(nodes_elem, f"{prefix}Node{i}", pos, settings.model_node_mass, settings.model_node_fixed, p_n, c_names)
                if settings.model_include_necessary_tri_body:
                    def w_tri(n1,n2,n3, suffix): ET.SubElement(figs_elem, f"{prefix}Foot-Triangle{suffix}", Type="Triangle", Shading="0", Node1=n1, Node2=n2, Node3=n3)
                    w_tri("NHeel_1","NToe_1","NAnkle_1","1_1"); w_tri("NToeS_1","NToe_1","NHeel_1","2_1")
                    w_tri("NHeel_2","NToe_2","NAnkle_2","1_2"); w_tri("NHeel_2","NToeS_2","NToe_2","2_2"); w_tri("NToeS_2","NToe_2","NAnkle_2","3_2")
            elif expected_type == "MODEL":
                pivot_idx = get_cloth_indices(o, settings.model_pivot) if settings.model_use_pivot else set()
                name_map = {}
                for v in verts:
                    pos = o.matrix_world @ v.co
                    is_cloth, is_piv = v.index in cloth_idx, v.index in pivot_idx
                    is_macro = v.index in macro_idx and custom_p_nodes and custom_child_names
                    name = "NPivot" if is_piv else f"{prefix}Node{start_node + v.index}"
                    name_map[v.index] = name
                    
                    if is_macro and not is_cloth:
                        write_macronode(nodes_elem, name, pos, settings.model_node_mass, settings.model_node_fixed, custom_p_nodes, custom_child_names)
                    else:
                        attribs = {"Type": "Node", "X": str(pos.x), "Y": str(pos.z), "Z": str(-pos.y),
                                   "Mass": str(settings.model_export_cloth_mass if is_cloth else settings.model_node_mass),
                                   "Fixed": "0" if is_cloth else ("1" if settings.model_node_fixed else "0"),
                                   "PinFixed": "0", "Visible": "1", "Passive": "0", "Cloth": "1" if is_cloth else "0",
                                   "Collisible": "0" if is_cloth else ("1" if settings.model_node_collisible else "0")}
                        if is_cloth: attribs.update({"Attenuation": f"{settings.model_export_cloth_attenuation:.2f}", "Rank": "0"})
                        ET.SubElement(nodes_elem, name, **attribs)
                store_edge(context, edges, verts, expected_type, edges_elem, start_edge, start_node, name_map)
                store_face(context, faces, verts, expected_type, figs_elem, start_tri, start_node, name_map)
            else:
                store_edge(context, edges, verts, expected_type, edges_elem, start_edge, start_node)
                start_edge += len(edges)
                store_face(context, faces, verts, expected_type, figs_elem, start_tri, start_node)
                start_tri += len(faces)
                start_node = process_object_nodes(o, verts, nodes_elem, start_node, prefix, settings, cloth_idx, p_nodes, child_reqs, macro_idx, custom_p_nodes, custom_child_names)
            return True

        if m_type == "MODEL": process_mesh_export(obj, "MODEL", [], settings.model_export_cloth_general_folder)
        elif m_type == "HEAD_GEAR": process_mesh_export(obj, "HEAD_GEAR", ["NTop", "NHeadS_2", "NHeadS_1", "NHeadF"], settings.model_export_cloth_general_folder)
        elif m_type == "BODY_GEAR": process_mesh_export(obj, "BODY_GEAR", [], settings.model_export_cloth_general_folder)
        elif m_type == "WEAPON":
            process_mesh_export(settings.weapon_object_1, "WEAPON", ["Weapon-Node4_1","Weapon-Node3_1","Weapon-Node2_1","Weapon-Node1_1"], settings.model_export_cloth_weapon1_folder)
            process_mesh_export(settings.weapon_object_2, "WEAPON", ["Weapon-Node4_2","Weapon-Node3_2","Weapon-Node2_2","Weapon-Node1_2"], settings.model_export_cloth_weapon2_folder)
            if settings.model_include_attack_edges:
                process_mesh_export(settings.model_attack_edges_object_1, "WEAPON", ["Weapon-Node4_1","Weapon-Node3_1","Weapon-Node2_1","Weapon-Node1_1"], "", is_attack=True)
                process_mesh_export(settings.model_attack_edges_object_2, "WEAPON", ["Weapon-Node4_2","Weapon-Node3_2","Weapon-Node2_2","Weapon-Node1_2"], "", is_attack=True)
        elif m_type == "FOOT_GEAR":
            process_mesh_export(settings.foot_object_1, "FOOT_GEAR", ["NToeS_1", "NToe_1", "NHeel_1", "NAnkle_1"], settings.model_export_cloth_foot1_folder)
            process_mesh_export(settings.foot_object_2, "FOOT_GEAR", ["NToeS_2", "NToe_2", "NHeel_2", "NAnkle_2"], settings.model_export_cloth_foot2_folder)
        elif m_type == "RANGED":
            process_mesh_export(obj, "RANGED", ["Ranged-Node1_1","Ranged-Node2_1","Ranged-Node3_1","Ranged-Node4_1"], settings.model_export_cloth_general_folder)
            if settings.model_include_attack_edges:
                process_mesh_export(settings.model_attack_edges_object_1, "RANGED", ["Ranged-Node1_1","Ranged-Node2_1","Ranged-Node3_1","Ranged-Node4_1"], "", is_attack=True, is_ranged=True)
        
        # Capsules
        if settings.model_export_capsules and settings.model_export_capsules_folder:
            for obj_cap in settings.model_export_capsules_folder.objects:
                if obj_cap.type != 'MESH': continue
                mod = next((m for m in obj_cap.modifiers if m.type == 'NODES' and m.node_group), None)
                if not mod: continue
                
                try:
                    e1 = mod[mod.node_group.interface.items_tree["End1"].identifier]
                    e2 = mod[mod.node_group.interface.items_tree["End2"].identifier]
                    m1 = mod[mod.node_group.interface.items_tree["Margin1"].identifier]
                    m2 = mod[mod.node_group.interface.items_tree["Margin2"].identifier]
                    rad = mod[mod.node_group.interface.items_tree["Radius"].identifier]
                except: continue
                
                if not e1 or not e2: continue
                edge_val = mod[mod.node_group.interface.items_tree["Edge"].identifier] if settings.model_export_capsules_predefined else None
                
                if not settings.model_export_capsules_predefined:
                    for e in edges_elem:
                        if e.attrib.get('End1') == e1.name and e.attrib.get('End2') == e2.name: edge_val = e.tag; break
                        elif e.attrib.get('End2') == e1.name and e.attrib.get('End1') == e2.name: edge_val = e.tag; m1, m2 = m2, m1; break
                if edge_val:
                    ET.SubElement(figs_elem, obj_cap.name, Type="Capsule", Edge=edge_val, Radius1=f"{rad:.2f}", Radius2=f"{rad:.2f}", Margin1=str(m1), Margin2=str(m2))

        filepath = self.filepath if self.filepath.lower().endswith(".xml") else self.filepath + ".xml"
        with open(filepath, "w", encoding="utf-8") as f: f.write(minidom.parseString(ET.tostring(root, encoding="unicode")).toprettyxml(indent="  "))
        self.report({'INFO'}, f"Model exported to {filepath}")
        return {'FINISHED'}

    def invoke(self, context, event):
        self.filepath = bpy.path.abspath("//") + "exported_model.xml"
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

class AddNodesOperator(bpy.types.Operator):
    bl_idname = "model.add_nodes"
    bl_label = "Import Nodes"
    bl_description = "Add nodes from the Model XML into Blender as spheres"
    bl_options = {'REGISTER', 'UNDO'}
    
    def execute(self, context):
        model_path = None
        model_path_unconvert = context.scene.gymnast_normal_xml
        
        if model_path_unconvert:
            model_path = bpy.path.abspath(model_path_unconvert)
        else:
            self.report({'ERROR'}, "No model XML file selected")
            return {'CANCELLED'}
        
        if not os.path.exists(model_path):
            raise Exception("Model XML path is missing or invalid.")
        
        props = context.scene.gymnast_tool_model_props

        # Create or get the "Model" collection
        model_collection = bpy.data.collections.get("Model")
        if not model_collection:
            model_collection = bpy.data.collections.new("Model")
            context.scene.collection.children.link(model_collection)

        # Create or get the child collection named after the model file
        model_name = os.path.basename(model_path).split('.')[0]
        child_collection = bpy.data.collections.get(model_name)
        if not child_collection:
            child_collection = bpy.data.collections.new(model_name)
            model_collection.children.link(child_collection)

        # Create or get the "Nodes_" collection
        nodes_collection_name = f"Nodes_{model_name}"
        nodes_collection = bpy.data.collections.get(nodes_collection_name)
        if not nodes_collection:
            nodes_collection = bpy.data.collections.new(nodes_collection_name)
            child_collection.children.link(nodes_collection)

        # Parse the XML file
        tree = ET.parse(model_path)
        root = tree.getroot()

        nodes_section = root.find('Nodes')
        if nodes_section is None:
            self.report({'ERROR'}, "No <Nodes> section found in XML")
            return {'CANCELLED'}
        
        # Store node positions for referencing
        node_positions = {}
        
        # Pass 1: Parse all Nodes
        for node in nodes_section:
            ntype = node.get('Type')
            if ntype in ['Node', 'CenterOfMass'] or (ntype == 'MacroNode' and not props.calculate_macronode):
                x, y, z = safe_float(node.get('X')), safe_float(node.get('Y')), safe_float(node.get('Z'))
                node_positions[node.tag] = (x, y, z)
                
        # Pass 2: Calculate MacroNodes via LCCs if enabled
        if props.calculate_macronode:
            for node in nodes_section:
                if node.get('Type') == 'MacroNode':
                    lcc_pos = [0.0, 0.0, 0.0]
                    for i in range(1, 5):
                        child_name = node.get(f'ChildNode{i}')
                        lcc_val = node.get(f'LCC{i}')
                        
                        if child_name and child_name != "Null" and lcc_val:
                            lcc = safe_float(lcc_val)
                            
                            if child_name in node_positions:
                                cx, cy, cz = node_positions[child_name]
                                lcc_pos[0] += cx * lcc
                                lcc_pos[1] += cy * lcc
                                lcc_pos[2] += cz * lcc
                            else:
                                blender_obj = bpy.data.objects.get(child_name)
                                if blender_obj:
                                    cx, cz, cy = blender_obj.location  # Blender is x,z,y
                                    lcc_pos[0] += cx * lcc
                                    lcc_pos[1] += cy * lcc
                                    lcc_pos[2] += -cz * lcc
                                    
                    node_positions[node.tag] = tuple(lcc_pos)

        # Generate the Geometry
        if props.import_node_as_vertex:
            # Add all points into a single mesh object
            mesh = bpy.data.meshes.new(f"Nodes_{model_name}")
            obj = bpy.data.objects.new(f"Nodes_{model_name}", mesh)
            nodes_collection.objects.link(obj)
            
            verts = [(x, -z, y) for name, (x, y, z) in node_positions.items()]
            mesh.from_pydata(verts, [], [])
            mesh.update()
            
        else:
            # Create one single UV Sphere mesh data block using BMesh
            bm = bmesh.new()
            bmesh.ops.create_uvsphere(bm, u_segments=16, v_segments=8, radius=1.0)
            sphere_mesh = bpy.data.meshes.new("NodeSphereData")
            bm.to_mesh(sphere_mesh)
            bm.free()

            # Quickly instantiate a new object pointing to that shared mesh for each node
            for node_name, (x, y, z) in node_positions.items():
                sphere = bpy.data.objects.new(node_name, sphere_mesh)
                sphere.location = (x, -z, y)
                sphere.scale = (1, 1, 1)
                sphere.display.show_shadows = False
                nodes_collection.objects.link(sphere)

        self.report({'INFO'}, "Nodes added successfully")
        return {'FINISHED'}

class AddEdgesOperator(bpy.types.Operator):
    bl_idname = "model.add_edges"
    bl_label = "Import Nodes and Edges"
    bl_description = "Add nodes as vertices and edges from the Model XML into blender as an object."
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        props = context.scene.gymnast_tool_model_props
        dependencies_path = bpy.path.abspath(context.scene.gymnast_dependencies_xml) if context.scene.gymnast_dependencies_xml else None
        model_path_unconvert = context.scene.gymnast_normal_xml
        
        if model_path_unconvert:
            model_path = bpy.path.abspath(model_path_unconvert)
        else:
            self.report({'ERROR'}, "No model XML file selected")
            return {'CANCELLED'}
            
        if not os.path.exists(model_path):
            raise Exception("Model XML path is missing or invalid.")
            
        model_name = os.path.basename(model_path).split('.')[0]

        # Parse the XML files
        tree = ET.parse(model_path)
        root = tree.getroot()

        nodes_section = root.find('Nodes')
        edges_section = root.find('Edges')

        if nodes_section is None:
            self.report({'ERROR'}, "No Nodes section found in XML")
            return {'CANCELLED'}

        # Preload dependencies if requested to prevent MacroNodes and Edges from breaking
        dep_nodes_dict = {}
        if props.model_use_dependencies and dependencies_path and os.path.exists(dependencies_path):
            dep_tree = ET.parse(dependencies_path)
            for d_node in dep_tree.getroot().find('Nodes') or []:
                dep_nodes_dict[d_node.tag] = d_node
        
        verts = []
        node_name_to_index = {}
        nodes = {}
        vertex_counter = 0

        # First pass: regular nodes and CenterOfMass
        macro_nodes = []
        for node in nodes_section:
            node_type = node.get('Type')
            node_name = node.tag

            if node_type in ['Node', 'CenterOfMass'] or (node_type == 'MacroNode' and not props.calculate_macronode):
                x, y, z = safe_float(node.get('X')), safe_float(node.get('Y')), safe_float(node.get('Z'))
                nodes[node_name] = (x, y, z)
                verts.append((x, -z, y))
                node_name_to_index[node_name] = vertex_counter
                vertex_counter += 1
            elif node_type == 'MacroNode' and props.calculate_macronode:
                macro_nodes.append(node)

        # Second pass: Calculate macro nodes via LCCs
        if props.calculate_macronode:
            for node in macro_nodes:
                node_name = node.tag
                lcc_pos = [0.0, 0.0, 0.0]

                for i in range(1, 5):
                    child_name = node.get(f'ChildNode{i}')
                    lcc_val = node.get(f'LCC{i}')
                    
                    if child_name and child_name != "Null" and lcc_val:
                        lcc = safe_float(lcc_val)

                        if child_name in nodes:
                            cx, cy, cz = nodes[child_name]
                        elif child_name in dep_nodes_dict:
                            d_node = dep_nodes_dict[child_name]
                            cx, cy, cz = safe_float(d_node.get('X')), safe_float(d_node.get('Y')), safe_float(d_node.get('Z'))
                        else:
                            blender_obj = bpy.data.objects.get(child_name)
                            if blender_obj:
                                cx, cz, cy = blender_obj.location
                                cy = -cy
                            else:
                                continue
                            
                        lcc_pos[0] += cx * lcc
                        lcc_pos[1] += cy * lcc
                        lcc_pos[2] += cz * lcc

                x, y, z = lcc_pos
                nodes[node_name] = (x, y, z)
                verts.append((x, -z, y))
                node_name_to_index[node_name] = vertex_counter
                vertex_counter += 1

        edges = []
        if edges_section is not None:
            for edge in edges_section:
                end1 = edge.get('End1')
                end2 = edge.get('End2')
                
                # Fallback to dependencies nodes if they are used as edge boundaries but weren't defined in the current XML
                for end in (end1, end2):
                    if end not in node_name_to_index and end in dep_nodes_dict:
                        d_node = dep_nodes_dict[end]
                        x, y, z = safe_float(d_node.get('X')), safe_float(d_node.get('Y')), safe_float(d_node.get('Z'))
                        verts.append((x, -z, y))
                        node_name_to_index[end] = vertex_counter
                        vertex_counter += 1

                if end1 in node_name_to_index and end2 in node_name_to_index:
                    edges.append((node_name_to_index[end1], node_name_to_index[end2]))
                    
        # Create the mesh
        mesh = bpy.data.meshes.new(f"Edges_{model_name}")
        mesh.from_pydata(verts, edges, [])
        mesh.update()

        # Create the object
        obj = bpy.data.objects.new(f"Edges_{model_name}", mesh)

        model_collection = bpy.data.collections.get("Model") or bpy.data.collections.new("Model")
        if model_collection.name not in context.scene.collection.children:
            context.scene.collection.children.link(model_collection)

        child_collection = bpy.data.collections.get(model_name) or bpy.data.collections.new(model_name)
        if child_collection.name not in model_collection.children:
            model_collection.children.link(child_collection)
            
        edges_collection_name = f"Edges_{model_name}"
        edges_collection = bpy.data.collections.get(edges_collection_name) or bpy.data.collections.new(edges_collection_name)
        if edges_collection.name not in child_collection.children:
            child_collection.children.link(edges_collection)

        edges_collection.objects.link(obj)

        self.report({'INFO'}, "Edges imported successfully")
        return {'FINISHED'}

class AddCapsulesOperator(bpy.types.Operator):
    bl_idname = "model.add_capsules"
    bl_label = "Import Capsules"
    bl_description = "Add Capsules from the model XML into Blender"
    bl_options = {'REGISTER', 'UNDO'}
    
    def execute(self, context):
        props = context.scene.gymnast_tool_model_props
        
        dependencies_path_unconvert = context.scene.gymnast_dependencies_xml
        model_path_unconvert = context.scene.gymnast_normal_xml
        
        dependencies_path = bpy.path.abspath(dependencies_path_unconvert) if dependencies_path_unconvert else None
        
        if model_path_unconvert:
            model_path = bpy.path.abspath(model_path_unconvert)
        else:
            self.report({'ERROR'}, "No model XML file selected")
            return {'CANCELLED'}
        
        NODE_GROUP_NAME = "Smooth Capsules"
        
        if not os.path.exists(model_path):
            raise Exception("Model XML path is missing or invalid.")
        
        if props.model_use_dependencies and not dependencies_path:
            raise Exception("Dependencies path is missing or invalid.")
        
        if props.model_use_dependencies and dependencies_path and not os.path.exists(dependencies_path):
            raise Exception("Dependencies XML path is missing or invalid.")
        
        def check_smooth_capsules():
            node_group = bpy.data.node_groups.get("Smooth Capsules")
            return node_group and node_group.bl_idname == 'GeometryNodeTree'
        
        def add_geometry_node():
            misc_collection = bpy.data.collections.get("Misc") or bpy.data.collections.new("Misc")
            if misc_collection.name not in context.scene.collection.children:
                context.scene.collection.children.link(misc_collection)
                
            mesh_name = "GeometryNodeHolder"
            mesh_data = bpy.data.meshes.new(mesh_name)
            geometryNodeHolder_obj = bpy.data.objects.new(mesh_name, mesh_data)
            misc_collection.objects.link(geometryNodeHolder_obj)
            
            node_tree_name = "Smooth Capsules"

            if node_tree_name not in bpy.data.node_groups:
                node_tree = bpy.data.node_groups.new(name=node_tree_name, type='GeometryNodeTree')
            else:
                node_tree = bpy.data.node_groups[node_tree_name]
                
            node_tree.is_modifier = True
            
            modifier = geometryNodeHolder_obj.modifiers.new(name="GeometryNodes", type='NODES')
            modifier.node_group = node_tree

            node_tree.nodes.clear()
            nodes = node_tree.nodes
            links = node_tree.links
            interface = node_tree.interface
            
            def add_input(name, socket_type, default=None, subtype=None, min_val=None, max_val=None):
                socket = interface.new_socket(name=name, in_out='INPUT', socket_type=socket_type)
                if socket_type == 'NodeSocketFloat':
                    if default is not None: socket.default_value = default
                    if subtype: socket.subtype = subtype
                    if min_val is not None: socket.min_value = min_val
                    if max_val is not None: socket.max_value = max_val
            
            group_input = nodes.new("NodeGroupInput")
            group_input.location = (0, 0)
            add_input("End1", "NodeSocketObject")
            add_input("End2", "NodeSocketObject")
            add_input("Margin1", "NodeSocketFloat", default=0.0, subtype='FACTOR', min_val=0.0, max_val=1.0)
            add_input("Margin2", "NodeSocketFloat", default=1.0, subtype='FACTOR', min_val=0.0, max_val=1.0)
            add_input("Radius",  "NodeSocketFloat", default=0.0, subtype='DISTANCE', min_val=0.0)
            add_input("Edge",  "NodeSocketString")
            
            group_input_2 = nodes.new(type=group_input.bl_idname)
            group_input_2.location = (-190, -500)
            
            group_output = nodes.new("NodeGroupOutput")
            group_output.location = (1750, 0)
            if "Geometry" not in [s.name for s in node_tree.interface.items_tree]:
                interface.new_socket(name="Geometry", in_out='OUTPUT', socket_type='NodeSocketGeometry')
            
            obj_info1 = nodes.new("GeometryNodeObjectInfo")
            obj_info1.location = (200, 0)
            obj_info1.inputs["As Instance"].default_value = False

            obj_info2 = nodes.new("GeometryNodeObjectInfo")
            obj_info2.location = (200, -220)
            obj_info2.inputs["As Instance"].default_value = False
            
            shader_node_math_thing = nodes.new("ShaderNodeMath")
            shader_node_math_thing.location = (450, -220)
            shader_node_math_thing.operation = 'SUBTRACT'
            shader_node_math_thing.use_clamp = True
            shader_node_math_thing.inputs[0].default_value = 1
            
            curve_line = nodes.new("GeometryNodeCurvePrimitiveLine")
            curve_line.location = (450, 0)
            
            trim_curve = nodes.new("GeometryNodeTrimCurve")
            trim_curve.location = (650, 0)
            
            curve_circle = nodes.new("GeometryNodeCurvePrimitiveCircle")
            curve_circle.location = (800, 0)
            curve_circle.inputs["Resolution"].default_value = 16
            
            curve_to_mesh = nodes.new("GeometryNodeCurveToMesh")
            curve_to_mesh.location = (1000, 0)
            curve_to_mesh.inputs["Fill Caps"].default_value = False
            
            store_named_attribute_bevelinputcurve = nodes.new("GeometryNodeStoreNamedAttribute")
            store_named_attribute_bevelinputcurve.location = (1250, 0)
            store_named_attribute_bevelinputcurve.data_type = 'FLOAT_VECTOR'
            store_named_attribute_bevelinputcurve.domain = 'POINT'
            store_named_attribute_bevelinputcurve.inputs["Name"].default_value = "nor"
            
            normal_bevelinputcurve = nodes.new("GeometryNodeInputNormal")
            normal_bevelinputcurve.location = (1000, -200)
            
            uv_sphere = nodes.new("GeometryNodeMeshUVSphere")
            uv_sphere.location = (0, -500)
            uv_sphere.inputs["Segments"].default_value = 16
            uv_sphere.inputs["Rings"].default_value = 8
            
            store_named_attribute_halfspherecap = nodes.new("GeometryNodeStoreNamedAttribute")
            store_named_attribute_halfspherecap.location = (200, -500)
            store_named_attribute_halfspherecap.data_type = 'FLOAT_VECTOR'
            store_named_attribute_halfspherecap.domain = 'POINT'
            store_named_attribute_halfspherecap.inputs["Name"].default_value = "nor"
            
            normal_halfspherecap = nodes.new("GeometryNodeInputNormal")
            normal_halfspherecap.location = (0, -680)
            
            delete_geometry = nodes.new("GeometryNodeDeleteGeometry")
            delete_geometry.location = (450, -500)
            delete_geometry.domain = 'FACE'
            delete_geometry.mode = 'ALL'
            
            input_position = nodes.new("GeometryNodeInputPosition")
            input_position.location = (0, -800)

            separate_xyz = node_tree.nodes.new("ShaderNodeSeparateXYZ")
            separate_xyz.location = (180, -800)
            
            compare_node = node_tree.nodes.new("FunctionNodeCompare")
            compare_node.location = (350, -800)
            compare_node.data_type = 'FLOAT'
            compare_node.operation = 'LESS_THAN'
            
            set_shade_smooth = nodes.new("GeometryNodeSetShadeSmooth")
            set_shade_smooth.location = (710, -500)
            
            end_point_selection_halfspherecap = nodes.new("GeometryNodeCurveEndpointSelection")
            end_point_selection_halfspherecap.location = (880, -500)
            
            instance_on_points = nodes.new("GeometryNodeInstanceOnPoints")
            instance_on_points.location = (1100, -500)
            
            store_named_attribute_halfspherecap2 = nodes.new("GeometryNodeStoreNamedAttribute")
            store_named_attribute_halfspherecap2.location = (1350, -500)
            store_named_attribute_halfspherecap2.data_type = 'FLOAT_VECTOR'
            store_named_attribute_halfspherecap2.domain = 'INSTANCE'
            store_named_attribute_halfspherecap2.inputs["Name"].default_value = "inst_rot"
            
            instance_rotation = nodes.new("GeometryNodeInputInstanceRotation")
            instance_rotation.location = (1350, -780)
            
            euler_to_rotation = nodes.new("FunctionNodeEulerToRotation")
            euler_to_rotation.location = (1020, -300)
            
            curve_tangent = nodes.new("GeometryNodeInputTangent")
            curve_tangent.location = (0, -1500)
            
            end_point_selection_aligncap = nodes.new("GeometryNodeCurveEndpointSelection")
            end_point_selection_aligncap.location = (200, -1400)
            end_point_selection_aligncap.inputs["End Size"].default_value = 0
            
            vector_math = nodes.new("ShaderNodeVectorMath")
            vector_math.location = (200, -1650)
            vector_math.operation = 'SCALE'
            vector_math.inputs["Scale"].default_value = -1
            
            switch_node = nodes.new("GeometryNodeSwitch")
            switch_node.location = (420, -1500)
            switch_node.input_type = 'VECTOR'
            
            align_rotation_to_vector_1 = nodes.new("FunctionNodeAlignRotationToVector")
            align_rotation_to_vector_1.location = (630, -1500)
            align_rotation_to_vector_1.axis = 'Z'
            align_rotation_to_vector_1.pivot_axis = 'AUTO'
            
            align_rotation_to_vector_2 = nodes.new("FunctionNodeAlignRotationToVector")
            align_rotation_to_vector_2.location = (820, -1500)
            align_rotation_to_vector_2.axis = 'X'
            align_rotation_to_vector_2.pivot_axis = 'AUTO'
            
            normal_aligncap = nodes.new("GeometryNodeInputNormal")
            normal_aligncap.location = (420, -1800)
            
            join_geo = nodes.new("GeometryNodeJoinGeometry")
            join_geo.location = (1500, 0)
            
            links.new(group_input.outputs["End1"], obj_info1.inputs[0])
            links.new(group_input.outputs["End2"], obj_info2.inputs[0])
            links.new(obj_info1.outputs["Location"], curve_line.inputs["Start"])
            links.new(obj_info2.outputs["Location"], curve_line.inputs["End"])
            links.new(curve_line.outputs["Curve"], trim_curve.inputs["Curve"])
            links.new(group_input.outputs["Margin1"], trim_curve.inputs["Start"])
            links.new(group_input.outputs["Margin2"], shader_node_math_thing.inputs[1])
            links.new(shader_node_math_thing.outputs["Value"], trim_curve.inputs["End"])
            links.new(trim_curve.outputs["Curve"], curve_to_mesh.inputs["Curve"])
            links.new(group_input.outputs["Radius"], curve_circle.inputs["Radius"])
            links.new(curve_circle.outputs["Curve"], curve_to_mesh.inputs["Profile Curve"])
            links.new(curve_to_mesh.outputs["Mesh"], store_named_attribute_bevelinputcurve.inputs["Geometry"])
            links.new(normal_bevelinputcurve.outputs["Normal"], store_named_attribute_bevelinputcurve.inputs["Value"])
            links.new(store_named_attribute_bevelinputcurve.outputs["Geometry"], join_geo.inputs["Geometry"])
            
            links.new(group_input_2.outputs["Radius"], uv_sphere.inputs["Radius"])
            links.new(uv_sphere.outputs["Mesh"], store_named_attribute_halfspherecap.inputs["Geometry"])
            links.new(normal_halfspherecap.outputs["Normal"], store_named_attribute_halfspherecap.inputs["Value"])
            links.new(store_named_attribute_halfspherecap.outputs["Geometry"], delete_geometry.inputs["Geometry"])
            links.new(input_position.outputs["Position"], separate_xyz.inputs["Vector"])
            links.new(separate_xyz.outputs["Z"], compare_node.inputs["A"])
            links.new(compare_node.outputs["Result"], delete_geometry.inputs["Selection"])
            links.new(delete_geometry.outputs["Geometry"], set_shade_smooth.inputs["Geometry"])
            links.new(set_shade_smooth.outputs["Geometry"], instance_on_points.inputs["Instance"])
            links.new(end_point_selection_halfspherecap.outputs["Selection"], instance_on_points.inputs["Selection"])
            links.new(trim_curve.outputs["Curve"], instance_on_points.inputs["Points"])
            links.new(euler_to_rotation.outputs["Rotation"], instance_on_points.inputs["Rotation"])
            links.new(align_rotation_to_vector_2.outputs["Rotation"], euler_to_rotation.inputs["Euler"])
            links.new(instance_on_points.outputs["Instances"], store_named_attribute_halfspherecap2.inputs["Geometry"])
            links.new(instance_rotation.outputs["Rotation"], store_named_attribute_halfspherecap2.inputs["Value"])
            links.new(store_named_attribute_halfspherecap2.outputs["Geometry"], join_geo.inputs["Geometry"])
            
            links.new(curve_tangent.outputs["Tangent"], vector_math.inputs["Vector"])
            links.new(curve_tangent.outputs["Tangent"], switch_node.inputs["False"])
            links.new(vector_math.outputs["Vector"], switch_node.inputs["True"])
            links.new(end_point_selection_aligncap.outputs["Selection"], switch_node.inputs["Switch"])
            links.new(switch_node.outputs["Output"], align_rotation_to_vector_1.inputs["Vector"])
            links.new(align_rotation_to_vector_1.outputs["Rotation"], align_rotation_to_vector_2.inputs["Rotation"])
            links.new(normal_aligncap.outputs["Normal"], align_rotation_to_vector_2.inputs["Vector"])
            
            links.new(join_geo.outputs["Geometry"], group_output.inputs["Geometry"])
            node_tree.use_fake_user = True
        
        def load_edges_from_xml(path):
            tree = ET.parse(path)
            root = tree.getroot()
            edge_dict = {}
            edges_elem = root.find("Edges")
            if edges_elem is not None:
                for edge in edges_elem:
                    edge_dict[edge.tag] = {
                        'End1': edge.attrib['End1'],
                        'End2': edge.attrib['End2']
                    }
            return root, edge_dict
        
        if not check_smooth_capsules():
            add_geometry_node()
        
        model_filename = os.path.splitext(os.path.basename(model_path))[0]
        model_collection_name = model_filename
        capsule_collection_name = f"Capsules_{model_filename}"
        
        model_root = bpy.data.collections.get("Model") or bpy.data.collections.new("Model")
        if model_root.name not in context.scene.collection.children:
            context.scene.collection.children.link(model_root)

        model_sub = bpy.data.collections.get(model_collection_name) or bpy.data.collections.new(model_collection_name)
        if model_sub.name not in model_root.children:
            model_root.children.link(model_sub)

        capsule_collection = bpy.data.collections.get(capsule_collection_name) or bpy.data.collections.new(capsule_collection_name)
        if capsule_collection.name not in model_sub.children:
            model_sub.children.link(capsule_collection)
        
        root_primary, edges_primary = load_edges_from_xml(model_path)
        if props.model_use_dependencies and dependencies_path is not None and os.path.exists(dependencies_path):
            _, edges_secondary = load_edges_from_xml(dependencies_path)
            edges = {**edges_secondary, **edges_primary}
        else:
            edges = dict(edges_primary)
        
        node_group = bpy.data.node_groups.get(NODE_GROUP_NAME)
        if not node_group:
            raise Exception(f"Geometry Node Group '{NODE_GROUP_NAME}' not found.")
            
        socket_map = {socket.name: socket.identifier for socket in node_group.interface.items_tree if socket.in_out == 'INPUT'}
        
        # Create a single lightweight mesh data block to be shared among all capsule objects
        # We MUST add at least one dummy vertex, otherwise Blender's Depsgraph will ignore the object 
        # and the Geometry Nodes won't evaluate or render on the screen
        shared_capsule_mesh = bpy.data.meshes.new("CapsuleBaseMesh")
        shared_capsule_mesh.from_pydata([(0.0, 0.0, 0.0)], [], [])
        shared_capsule_mesh.update()

        capsules = [fig for fig in root_primary.find("Figures") if fig.attrib.get("Type") == "Capsule"]
        
        for capsule in capsules:
            capsule_name = capsule.tag
            edge_name = capsule.attrib["Edge"]
            
            edge_info = edges.get(edge_name)
            if not edge_info: continue

            end1_obj = bpy.data.objects.get(edge_info["End1"])
            end2_obj = bpy.data.objects.get(edge_info["End2"])
            if not end1_obj or not end2_obj: continue

            radius1 = safe_float(capsule.attrib.get("Radius1", 1.0))
            margin1 = safe_float(capsule.attrib.get("Margin1", 0.0))
            margin2 = safe_float(capsule.attrib.get("Margin2", 0.0))

            # Instantiate the object directly sharing the dummy vertex mesh
            capsule_obj = bpy.data.objects.new(capsule_name, shared_capsule_mesh)
            capsule_collection.objects.link(capsule_obj)

            modifier = capsule_obj.modifiers.new(name="GeometryNodes", type='NODES')
            modifier.node_group = node_group

            try:
                modifier[socket_map["End1"]] = end1_obj
                modifier[socket_map["End2"]] = end2_obj
                modifier[socket_map["Margin1"]] = margin1
                modifier[socket_map["Margin2"]] = margin2
                modifier[socket_map["Radius"]] = radius1
                modifier[socket_map["Edge"]] = edge_name
            except KeyError as e:
                print(f"Socket name missing in Geometry Node Group: {e}")
        
        # Force the viewport to update and draw the new capsules
        context.view_layer.update()
        
        self.report({'INFO'}, f"Imported {len(capsules)} capsules.")
        return {'FINISHED'}
 
class SetOrientation(bpy.types.Operator):
    bl_idname = "model.set_orientation"
    bl_label = "Set Orientation"
    bl_description = "Set the Orientation of the Object's Origin"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        st = context.scene.gymnast_tool_model_props
        m_type = st.model_type_export
        adv = st.model_is_advanced
        
        # --- Handle Body Gear ---
        if not adv and m_type == 'BODY_GEAR':
            model_path_unconvert = context.scene.gymnast_normal_xml
            if not model_path_unconvert:
                self.report({'ERROR'}, "No model XML file selected")
                return {'CANCELLED'}
                
            model_path = bpy.path.abspath(model_path_unconvert)
            model_name = os.path.basename(model_path).split('.')[0]
            
            # Setup Collections for Hooks
            model_collection = bpy.data.collections.get("Model") or bpy.data.collections.new("Model")
            if model_collection.name not in context.scene.collection.children:
                context.scene.collection.children.link(model_collection)
                
            child_collection = bpy.data.collections.get(model_name) or bpy.data.collections.new(model_name)
            if child_collection.name not in model_collection.children:
                model_collection.children.link(child_collection)
                
            hook_collection_name = f"Hook_{model_name}"
            hook_collection = bpy.data.collections.get(hook_collection_name) or bpy.data.collections.new(hook_collection_name)
            if hook_collection.name not in child_collection.children:
                child_collection.children.link(hook_collection)
                
            # Alignment configurations based on Body Gear section
            alignment_profiles = {
                'CHEST': {'copy_location': "NChest", 'track_1': "NNeck", 'track_2': "NChestF", 'track_3': "NChestS_2"},
                'STOMACH': {'copy_location': "NStomach", 'track_1': "NChest", 'track_2': "NStomachF", 'track_3': "NStomachS_2"},
                'HIP': {'copy_location': "NPivot", 'track_1': "NStomach", 'track_2': "NPelvisF", 'track_3': "NHip_2"}
            }

            def create_hook(vgroup_name, hook_suffix, alignment_type):
                if not st.selected_object or vgroup_name not in [vg.name for vg in st.selected_object.vertex_groups]:
                    return

                profile = alignment_profiles.get(alignment_type)
                if not profile:
                    return

                # Create Empty Object for the Hook
                hook_name = f"Hook_{hook_suffix}_{model_name}"
                empty = bpy.data.objects.new(hook_name, None)
                empty.empty_display_type = 'PLAIN_AXES'
                hook_collection.objects.link(empty)

                # Set up Constraints on the Empty
                if profile['copy_location']:
                    target = bpy.data.objects.get(profile['copy_location'])
                    if target:
                        constraint = empty.constraints.new(type='COPY_LOCATION')
                        constraint.target = target

                for axis, key in zip(['TRACK_Z', 'TRACK_Y', 'TRACK_X'], ['track_1', 'track_2', 'track_3']):
                    if st.model_align_flipped and axis == 'TRACK_X':
                        axis = 'TRACK_NEGATIVE_X'
                        
                    target_obj = bpy.data.objects.get(profile[key])
                    if target_obj:
                        constraint = empty.constraints.new(type='DAMPED_TRACK')
                        constraint.track_axis = axis
                        constraint.target = target_obj
                
                context.view_layer.update()
                
                # Apply Hook Modifier to Mesh
                mod = st.selected_object.modifiers.new(name=f"Hook_{vgroup_name}", type='HOOK')
                mod.object = empty
                mod.vertex_group = vgroup_name
                
                # Target the exact vertices inside the vertex group
                if st.selected_object.type == 'MESH':
                    mesh = st.selected_object.data
                    bm = bmesh.new()
                    bm.from_mesh(mesh)
                    group_index = st.selected_object.vertex_groups[vgroup_name].index
                    verts_in_group = [v.index for v in mesh.vertices if any(g.group == group_index for g in v.groups)]
                    if verts_in_group:
                        mod.vertex_indices_set(verts_in_group)
                    bm.free()

            # Hooks Generation
            create_hook("Armor_Top", "Top", st.model_body_top)
            create_hook("Armor_Middle", "Middle", st.model_body_middle)
            create_hook("Armor_Bottom", "Bottom", st.model_body_bottom)
            
            self.report({'INFO'}, "Body Gear constraints generated.")
            return {'FINISHED'}

        configs = []
        if adv or m_type == 'MODEL':
            if not st.model_orientation or not st.model_origin_object:
                self.report({'ERROR'}, "Both objects must be specified."); return {'CANCELLED'}
            sel = [o for o in context.selected_objects if o != st.model_orientation]
            if len(sel) != 2: self.report({'ERROR'}, "Select exactly two other objects."); return {'CANCELLED'}
            configs.append((st.model_orientation, st.model_origin_object, sel[0], sel[1], None, 'TRACK_X'))
            
        elif m_type == 'WEAPON':
            n_dict = get_child_nodes_dict(["Weapon-Node2_1","Weapon-Node1_1","Weapon-Node3_1","Weapon-Node4_1",
                                           "Weapon-Node2_2","Weapon-Node1_2","Weapon-Node3_2","Weapon-Node4_2"], self.report)
            if not n_dict: return {'CANCELLED'}
            if st.weapon_object_1: configs.append((st.weapon_object_1, bpy.data.objects["Weapon-Node2_1"], bpy.data.objects["Weapon-Node1_1"], bpy.data.objects["Weapon-Node3_1"], bpy.data.objects["Weapon-Node4_1"], 'TRACK_NEGATIVE_X'))
            if st.weapon_object_2: configs.append((st.weapon_object_2, bpy.data.objects["Weapon-Node2_2"], bpy.data.objects["Weapon-Node1_2"], bpy.data.objects["Weapon-Node3_2"], bpy.data.objects["Weapon-Node4_2"], 'TRACK_X'))
            
        elif m_type == 'FOOT_GEAR':
            n_dict = get_child_nodes_dict(["NToe_1","NHeel_1","NToeS_1","NToe_2","NHeel_2","NToeS_2"], self.report)
            if not n_dict: return {'CANCELLED'}
            if st.foot_object_1: configs.append((st.foot_object_1, bpy.data.objects["NToe_1"], bpy.data.objects["NHeel_1"], bpy.data.objects["NToeS_1"], None, 'TRACK_X'))
            if st.foot_object_2: configs.append((st.foot_object_2, bpy.data.objects["NToe_2"], bpy.data.objects["NHeel_2"], bpy.data.objects["NToeS_2"], None, 'TRACK_X'))
            
        elif m_type == 'HEAD_GEAR':
            if not st.selected_object: return {'CANCELLED'}
            n_dict = get_child_nodes_dict(["NHead","NTop","NHeadF","NHeadS_2"], self.report)
            if not n_dict: return {'CANCELLED'}
            configs.append((st.selected_object, bpy.data.objects["NHead"], bpy.data.objects["NTop"], bpy.data.objects["NHeadF"], bpy.data.objects["NHeadS_2"], 'TRACK_NEGATIVE_X' if st.model_align_flipped else 'TRACK_X'))
            
        elif m_type == 'RANGED':
            if not st.selected_object: return {'CANCELLED'}
            n_dict = get_child_nodes_dict(["Ranged-Node2_1","Ranged-Node1_1","Ranged-Node4_1","Ranged-Node3_1"], self.report)
            if not n_dict: return {'CANCELLED'}
            configs.append((st.selected_object, bpy.data.objects["Ranged-Node2_1"], bpy.data.objects["Ranged-Node1_1"], bpy.data.objects["Ranged-Node4_1"], bpy.data.objects["Ranged-Node3_1"], 'TRACK_X'))

        # alignments
        for obj, orig, tz, ty, tx, track_x in configs:
            if st.model_use_origin or not st.model_use_existing_object:
                translate_origin_to_target(obj, orig.location)
                if st.model_use_existing_object:
                    align_object_to_basis(obj, orig.location, tz, ty)
            
            if st.model_apply_constraint or m_type in {'WEAPON', 'FOOT_GEAR', 'HEAD_GEAR', 'RANGED'}:
                if not st.model_use_existing_object:
                    obj.matrix_world.translation = Vector((0,0,0))
                    align_object_to_basis(obj, orig.location, tz, ty)
                setup_tracking_constraints(obj, orig, tz, ty, tx, track_x, use_offset=not st.model_use_existing_object)

        self.report({'INFO'}, "Orientation applied successfully.")
        return {'FINISHED'}

class AddRuleOperator(bpy.types.Operator):
    bl_idname = "macro_rules.add_rule"
    bl_label = "Add Rule"
    bl_description = "Add a new group rule."
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        context.scene.macro_rules.add()
        context.scene.macro_rules_index = len(context.scene.macro_rules) - 1
        return {'FINISHED'}

class RemoveRuleOperator(bpy.types.Operator):
    bl_idname = "macro_rules.remove_rule"
    bl_label = "Remove Rule"
    bl_description = "Remove a group rule."
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        index = context.scene.macro_rules_index
        if index >= 0:
            context.scene.macro_rules.remove(index)
            context.scene.macro_rules_index = min(index, len(context.scene.macro_rules) - 1)
        return {'FINISHED'}

class AddTemplateGroupsOperator(bpy.types.Operator):
    bl_idname = "macro_rules.add_templates"
    bl_label = "Preset Groups"
    bl_description = "Add selected preset groups to the rule."
    bl_options = {'REGISTER', 'UNDO'}

    group_type: bpy.props.EnumProperty(
        name="Group Type",
        description="Choose which preset groups to add",
        items=[
            ('ARMOR', "Armor", "Add only armor groups"),
            ('WEAPON', "Weapon", "Add only weapon groups"),
            ('ALL', "All", "Add all template groups"),
        ],
        default='ALL'
    )

    def execute(self, context):
        scene = context.scene
        existing_groups = {item.group for item in scene.macro_rules}
        templates = []

        if self.group_type in {'ARMOR', 'ALL'}:
            templates.extend([
                ("Armor_Top", "NChestS_2,NChestF,NChestS_1,NNeck"),
                ("Armor_Middle", "NStomachS_2,NStomachF,NStomachS_1,NChest"),
                ("Armor_Bottom", "NHip_1,NPelvisF,NHip_2,NStomach"),
            ])

        if self.group_type in {'WEAPON', 'ALL'}:
            templates.extend([
                ("Weapon_1", "Weapon-Node4_1,Weapon-Node3_1,Weapon-Node2_1,Weapon-Node1_1"),
                ("Weapon_2", "Weapon-Node4_2,Weapon-Node3_2,Weapon-Node2_2,Weapon-Node1_2"),
            ])

        added = 0
        for grp, names in templates:
            if grp not in existing_groups:
                rule = scene.macro_rules.add()
                rule.group = grp
                rule.names = names
                added += 1

        self.report({'INFO'}, f"Added {added} template group(s)")
        return {'FINISHED'}

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)

class ClearMacroRulesOperator(bpy.types.Operator):
    bl_idname = "macro_rules.clear_rules"
    bl_label = "Clear All Rules"
    bl_description = "Remove all entries from the list"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        context.scene.macro_rules.clear()
        context.scene.macro_rules_index = 0
        return {'FINISHED'}


# #################### #
# Settings             
# #################### #

class GymnastToolModelSettings(bpy.types.PropertyGroup):
    # GENERAL SETTINGS
    model_string_name: bpy.props.StringProperty(
        name="Prefix",
        description="The name to add into each element's name in the XML such as Nodes, Edges and Figures.\nEx. 'Cloth-', 'CoolStaff-'",
    )
    model_type_export: bpy.props.EnumProperty(
        name="Type",
        description="Choose the type of model\nDefault: MODEL",
        items=[
            ('MODEL', "Model", "Normal model. (Mostly Used in Vector)"),
            ('HEAD_GEAR', "Head Gear", "Helm or Head accessory model."),
            ('BODY_GEAR', "Body Gear", "Armor or Body accessory model."),
            ('FOOT_GEAR', "Foot Gear", "Shoes or Footwear accessory model."),
            ('WEAPON', "Weapon", "Weapon Model. (SF2 Only)"),
            ('RANGED', "Ranged", "Ranged Weapon. (SF2 Only)")
        ],
        default='MODEL',
        update=refresh_enum
    )

    # OBJECT POINTERS
    selected_object: bpy.props.PointerProperty(
        name="Object",
        description="Select the object to convert into XML",
        type=bpy.types.Object,
        update=refresh_enum
    )
    weapon_object_1: bpy.props.PointerProperty(
        name="Weapon 1",
        description="Select the weapon that will be attach to the left hand",
        type=bpy.types.Object
    )
    weapon_object_2: bpy.props.PointerProperty(
        name="Weapon 2",
        description="Select the weapon that will be attach to the right hand",
        type=bpy.types.Object
    )
    foot_object_1: bpy.props.PointerProperty(
        name="Footwear 1",
        description="Select the footwear that will be attach to the right foot (Vector) or left foot (SF2)",
        type=bpy.types.Object
    )
    foot_object_2: bpy.props.PointerProperty(
        name="Footwear 2",
        description="Select the weapon that will be attach to the left foot (Vector) or right foot (SF2)",
        type=bpy.types.Object
    )

    # NODE & EDGE PROPERTIES
    model_node_mass: bpy.props.FloatProperty(
        name="Node's Mass", 
        description="Mass for every node", 
        default=1.0,
        min=0.0,
        max=10000.0,
        precision=2
    )
    model_node_collisible: bpy.props.BoolProperty(
        name="Node's Collisible", 
        description="Every node is collisible\nDefault: False", 
        default=False
    )
    model_node_fixed: bpy.props.BoolProperty(
        name="Node's Fixed", 
        description="Every node is fixed and will not move (Unless it's played by model animation)\nDefault: False", 
        default=False
    )
    model_edge_collisible: bpy.props.BoolProperty(
        name="Edge's Collisible", 
        description="Every edge is collisible\nDefault: False", 
        default=False
    )
    model_node_offset: bpy.props.IntProperty(
        name="Start Node",
        description="Starting number for node numbering\nDefault: 1",
        default=1,
        min=1
    )
    model_edge_offset: bpy.props.IntProperty(
        name="Start Edge",
        description="Starting number for edge numbering\nDefault: 1",
        default=1,
        min=1
    )
    model_tri_offset: bpy.props.IntProperty(
        name="Start Tri",
        description="Starting number for triangle numbering\nDefault: 1",
        default=1,
        min=1
    )

    # CLOTH SETTINGS
    model_export_cloth: bpy.props.BoolProperty(
        name="Export Cloth", 
        description="Specify node-cloth based on the assigned vertices in object's Vertex Group\nDefault: False", 
        default=False
    )
    model_export_cloth_attenuation: bpy.props.FloatProperty(
        name="Attenuation", 
        description="Controls how much a cloth node resists deformation.\n0 = Soft\n 1 = Stiff", 
        default=0,
        min=0.0,
        max=2.0,
        precision=2
    )
    model_export_cloth_mass: bpy.props.FloatProperty(
        name="Mass", 
        description="Mass for every cloth nodes", 
        default=0.1,
        min=0.0,
        max=10000.0,
        precision=2
    )
    model_export_cloth_general_folder: bpy.props.EnumProperty(
        name="Cloth Group",
        description="The Vertex Group containing the Object's Vertices marked as a cloth node.",
        items=get_general_vertex_groups
    )
    model_export_cloth_weapon1_folder: bpy.props.EnumProperty(
        name="Cloth Group 1",
        description="The Vertex Group containing the Weapon 1's Vertices marked as a cloth node.",
        items=get_weapon1_vertex_groups
    )
    model_export_cloth_weapon2_folder: bpy.props.EnumProperty(
        name="Cloth Group 2",
        description="The Vertex Group containing the Weapon 2's Vertices marked as a cloth node.",
        items=get_weapon2_vertex_groups
    )
    model_export_cloth_foot1_folder: bpy.props.EnumProperty(
        name="Cloth Group 1",
        description="The Vertex Group containing the Foot 1's Vertices marked as a cloth node.",
        items=get_foot1_vertex_groups
    )
    model_export_cloth_foot2_folder: bpy.props.EnumProperty(
        name="Cloth Group 2",
        description="The Vertex Group containing the Foot 2's Vertices marked as a cloth node.",
        items=get_foot2_vertex_groups
    )

    # CAPSULES SETTINGS
    model_export_capsules: bpy.props.BoolProperty(
        name="Export Capsules", 
        description="Includes capsules during the model exporting.\nDefault: False", 
        default=False
    )
    model_export_capsules_predefined: bpy.props.BoolProperty(
        name="Use Predefined Edge.", 
        description="Instead of finding suitable edge automatically during the export, it will use the name of the edge under the socket input.\nDefault: False", 
        default=False
    )
    model_export_capsules_folder: bpy.props.PointerProperty(
        name="Capsules Collection",
        description="The collection containing the Capsules ready to export.",
        type=bpy.types.Collection
    )

    # ALIGNMENT & ORIENTATION
    model_orientation: bpy.props.PointerProperty(
        name="Object",
        description="Select the object to set orientation (Must select two object in the scene to indicate the orientation.)",
        type=bpy.types.Object
    )
    model_use_origin: bpy.props.BoolProperty(
        name="Set Origin",
        description="Change the origin's location of the selected object and apply a ChildOf Constraint.\nDefault: False",
        default=False
    )
    model_origin_object: bpy.props.PointerProperty(
        name="Origin",
        description="The Object position to set to the selected object's origin.",
        type=bpy.types.Object
    )
    model_apply_constraint: bpy.props.BoolProperty(
        name="Add Constraint", 
        description="Add a Damped Track and Copy Location Constraint to the Object.\nThe first selected Object will be Z and second will be Y\nDefault: False", 
        default=False
    )
    model_align_flipped: bpy.props.BoolProperty(
        name="Flipped",
        description="Whether or not the damped track should be flipped.\nNormally, Vector and SF2 rig has swapped side, 1 will be swapped with 2 (Ex. NAnkle_1 --> NAnkle_2)\nVector = True, SF2 = False\nDefault: False",
        default=False
    )

    # BODY GEAR SPECIFIC
    model_body_top: bpy.props.EnumProperty(
        name="Top",
        description="Select the body area that the body's gear will be attached to.\nDefault: CHEST",
        items=[
            ('CHEST', "Chest", "Chest area or the upper torso part to the neck."),
            ('STOMACH', "Stomach", "Stomach area or the middle torso, between the chest and the hip."),
            ('HIP', "Hip", "Hip area or the part below the middle torso.")
        ],
        default='CHEST'
    )
    model_body_middle: bpy.props.EnumProperty(
        name="Middle",
        description="Select the body area that the body's gear will be attached to.\nDefault: STOMACH",
        items=[
            ('CHEST', "Chest", "Chest area or the upper torso part to the neck."),
            ('STOMACH', "Stomach", "Stomach area or the middle torso, between the chest and the hip."),
            ('HIP', "Hip", "Hip area or the part below the middle torso.")
        ],
        default='STOMACH'
    )
    model_body_bottom: bpy.props.EnumProperty(
        name="Bottom",
        description="Select the body area that the body's gear will be attached to.\nDefault: HIP",
        items=[
            ('CHEST', "Chest", "Chest area or the upper torso part to the neck."),
            ('STOMACH', "Stomach", "Stomach area or the middle torso, between the chest and the hip."),
            ('HIP', "Hip", "Hip area or the part below the middle torso.")
        ],
        default='HIP'
    )
    model_include_necessary_tri_body: bpy.props.BoolProperty(
        name="Include Foot Triangle (SF2)", 
        description="Normally, there's a visible hole on the side of the foot for SF2's rig. We can hide this with a triangle.\nDefault: False", 
        default=False
    )

    # ATTACK EDGES & MISC ADVANCED
    model_is_advanced: bpy.props.BoolProperty(
        name="Advanced Options", 
        description="Allows for a manual set-up with complicated models.", 
        default=False
    )
    model_use_existing_object: bpy.props.BoolProperty(
        name="Use Existing OBJ", 
        description="When importing Nekki's Model, the LCCs will already be applied to the MacroNode. This make it so that when:\n Enabled - Set Origin Alignment and Constraint to the Model will not change its position visually.\n Disabled - Set Model's alignment and constraint will change its position and orientation (Used for Model that's not attached originally).\nDefault: True", 
        default=True
    )
    calculate_macronode: bpy.props.BoolProperty(
        name="Apply LCCs", 
        description="Whether or not to apply LCC to MacroNode while importing.\nDefault: True", 
        default=True
    )
    model_include_attack_edges: bpy.props.BoolProperty(
        name="Add Attack Edges", 
        description="Edges for defining the damage part.\nDefault: True", 
        default=True
    )
    model_attack_edges_object_1: bpy.props.PointerProperty(
        name="Edges 1",
        description="Select the object to be referenced as attack edges for weapon 1.",
        type=bpy.types.Object
    )
    model_attack_edges_object_2: bpy.props.PointerProperty(
        name="Edges 2",
        description="Select the object to be referenced as attack edges for weapon 2.",
        type=bpy.types.Object
    )
    model_use_pivot: bpy.props.BoolProperty(
        name="Use Pivot",
        description="Whether or not to specify which vertex should be a Pivot Node.\nIf you disable this and try to load the model in, the game may crash.\nDefault: True",
        default=True
    )
    model_pivot: bpy.props.EnumProperty(
        name="Pivot",
        description="The Vertex Group containing the Object's Vertex that will be referenced as a Pivot Node\nUsually consisting of just 1 Vertex.",
        items=get_general_vertex_groups
    )
    model_use_dependencies: bpy.props.BoolProperty(
        name="Use Dependencies", 
        description="While exporting model, it will also search for the nodes inside the Dependencies XML.\nIf this is disabled, then it will only use the node inside the Model XML.\nDefault: True", 
        default=True
    )
    import_node_as_vertex: bpy.props.BoolProperty(
        name="Import Node as Vertex", 
        description="Importing node will add nodes as a vertices, instead of UV Sphere.\nNote: For Import Nodes.\nDefault: False", 
        default=False
    )
    add_vertex_group: bpy.props.BoolProperty(
        name="Add Vertex Group", 
        description="While importing XML to OBJ, it will also add an appropriate Vertex Group based on the Node and MacroNode's attributes.\nEx. Cloth Node and Armor Top, Middle, Bottom section.\nDefault: True", 
        default=True
    )
    add_vertex_group_include_cloth: bpy.props.BoolProperty(
        name="Include Cloth", 
        description="Add a Cloth Vertex Group during the conversion.\nDefault: True", 
        default=True
    )

    # CHILDNODE SETTINGS (WIP)
    model_custom_childnode: bpy.props.BoolProperty(
        name="Custom ChildNodes", 
        description="**WORK IN PROGRESS: CURRENTLY FIXED AT ONLY 4 CHILDNODE\nDefine a custom childnodes.\nDefault: False", 
        default=False
    )
    childnode_1_object: bpy.props.PointerProperty(
        name="Childnode 1",
        description="Select the object to be referenced as childnode 1 for Macronode.",
        type=bpy.types.Object
    )
    childnode_2_object: bpy.props.PointerProperty(
        name="Childnode 2",
        description="Select the object to be referenced as childnode 2 for Macronode.",
        type=bpy.types.Object
    )
    childnode_3_object: bpy.props.PointerProperty(
        name="Childnode 3",
        description="Select the object to be referenced as childnode 3 for Macronode.",
        type=bpy.types.Object
    )
    childnode_4_object: bpy.props.PointerProperty(
        name="Childnode 4",
        description="Select the object to be referenced as childnode 4 for Macronode.",
        type=bpy.types.Object
    )
    macronode_vertex_group: bpy.props.EnumProperty(
        name="Macronode",
        description="The Vertex Group containing the Object's Vertices that will be referenced as a Macronode.",
        items=get_general_vertex_groups
    )
    macronode_vertex_group_weapon_1: bpy.props.EnumProperty(
        name="Macronode 1",
        description="The Vertex Group containing the Object's Vertices that will be referenced as a Macronode for the Weapon 1.",
        items=get_weapon1_vertex_groups
    )
    macronode_vertex_group_weapon_2: bpy.props.EnumProperty(
        name="Macronode 2",
        description="The Vertex Group containing the Object's Vertices that will be referenced as a Macronode for the Weapon 2.",
        items=get_weapon2_vertex_groups
    )
    
class MacroRuleItem(bpy.types.PropertyGroup):
    group: bpy.props.StringProperty(name="Group", description="Name of the Vertex Group.")
    names: bpy.props.StringProperty(name="Names", description="Names of the 4 ChildNode separated by commas with no space.\nEx. NChestS_2,NChestF,NChestS_1,NNeck")
    
    
# #################### #
# Sideview Panel Menu  
# #################### #


class VIEW3D_PT_gymnast_model_panel(bpy.types.Panel):
    bl_label = "Model Tools"
    bl_idname = "VIEW3D_PT_gymnast_model_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Gymnast Tool Suite'
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        layout.prop(scene, "gymnast_dependencies_xml")
        layout.prop(scene, "gymnast_normal_xml")
        
        box = layout.box()
        box.label(text="Model Options", icon='OBJECT_DATA')
        box.operator("model.convert_xml", text="Convert XML to OBJ")
        box.operator("model.export_to_xml", text="Convert OBJ to XML")
        box.operator("model.add_nodes", text="Import Nodes")
        box.operator("model.add_edges", text="Import Edges")
        box.operator("model.add_capsules", text="Import Capsules")

class VIEW3D_PT_gymnast_model_settings(bpy.types.Panel):
    bl_label = "Settings"
    bl_idname = "VIEW3D_PT_gymnast_model_settings"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Gymnast Tool Suite'
    bl_parent_id = "VIEW3D_PT_gymnast_model_panel"
    bl_options = {'DEFAULT_CLOSED'}
    
    def draw(self, context):
        pass # Acts as a parent panel

class VIEW3D_PT_gymnast_model_settings_import(bpy.types.Panel):
    bl_label = "Import Settings"
    bl_idname = "VIEW3D_PT_gymnast_model_settings_import"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Gymnast Tool Suite'
    bl_parent_id = "VIEW3D_PT_gymnast_model_settings"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        props = context.scene.gymnast_tool_model_props
        scene = context.scene
        rules = scene.macro_rules
        
        layout = self.layout
        box = layout.box()
        
        box.label(text="Import Settings")
        box.prop(props, "calculate_macronode")
        box.prop(props, "model_use_dependencies")
        box.prop(props, "import_node_as_vertex")
        
        box.prop(props, "add_vertex_group")
        if props.add_vertex_group:
            row = layout.row()
            row.template_list("MACRO_UL_rules", "", scene, "macro_rules", scene, "macro_rules_index")
            
            col = row.column(align=True)
            col.operator("macro_rules.add_rule", icon='ADD', text="")
            col.operator("macro_rules.remove_rule", icon='REMOVE', text="")
            col.operator("macro_rules.add_templates", icon='PRESET', text="")
            col.operator("macro_rules.clear_rules", icon='TRASH', text="")
            layout.prop(props, "add_vertex_group_include_cloth")
            
            if scene.macro_rules_index >= 0 and len(rules) > 0:
                item = rules[scene.macro_rules_index]
                layout.prop(item, "group")
                layout.prop(item, "names")

class MACRO_UL_rules(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        layout.label(text=item.group)

class VIEW3D_PT_gymnast_model_settings_export(bpy.types.Panel):
    bl_label = "Export Settings"
    bl_idname = "VIEW3D_PT_gymnast_model_settings_export"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Gymnast Tool Suite'
    bl_parent_id = "VIEW3D_PT_gymnast_model_settings"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        props = context.scene.gymnast_tool_model_props
        model_type = props.model_type_export
        
        layout = self.layout
        box = layout.box()
        box.label(text="Export Settings")
        box.prop(props, "model_string_name")
        box.prop(props, "model_type_export")
        
        if model_type == 'MODEL':
            box.prop(props, "selected_object")
            box.prop(props, "model_node_mass")
            box.prop(props, "model_node_fixed")
            box.prop(props, "model_node_collisible")
            box.prop(props, "model_edge_collisible")
            box.prop(props, "model_use_pivot")
            if props.model_use_pivot:
                box.prop(props, "model_pivot")
        elif model_type == 'HEAD_GEAR':
            box.prop(props, "selected_object")
            box.prop(props, "model_node_mass")
            box.prop(props, "model_edge_collisible")
        elif model_type == 'BODY_GEAR':
            box.prop(props, "selected_object")
            box.prop(props, "model_node_mass")
            box.prop(props, "model_body_top")
            box.prop(props, "model_body_middle")
            box.prop(props, "model_body_bottom")
            box.prop(props, "model_edge_collisible")            
        elif model_type == 'WEAPON':
            box.prop(props, "weapon_object_1")
            box.prop(props, "weapon_object_2")
            box.prop(props, "model_node_mass")
            box.prop(props, "model_node_fixed")
            box.prop(props, "model_edge_collisible")
        elif model_type == 'FOOT_GEAR':
            box.prop(props, "foot_object_1")
            box.prop(props, "foot_object_2")
            box.prop(props, "model_node_mass")
            box.prop(props, "model_edge_collisible")
        elif model_type == 'RANGED':
            box.prop(props, "selected_object")
            box.prop(props, "model_node_mass")
            box.prop(props, "model_node_fixed")
            box.prop(props, "model_edge_collisible")
        
        if model_type in {'WEAPON', 'RANGED'}:
            box3 = layout.box()
            box3.label(text="Attack Edges")
            box3.prop(props, "model_include_attack_edges")
            if props.model_include_attack_edges:
                box3.prop(props, "model_attack_edges_object_1")
                if model_type == 'WEAPON':
                    box3.prop(props, "model_attack_edges_object_2")
        
        box2 = layout.box()
        box2.label(text="Additional Settings")
        
        if model_type == 'BODY_GEAR':
            box2.prop(props, "model_include_necessary_tri_body")
        
        box2.prop(props, "model_export_capsules")
        if props.model_export_capsules:
            box2.prop(props, "model_export_capsules_predefined")
            box2.prop(props, "model_export_capsules_folder")
            
        box2.prop(props, "model_export_cloth")
        if props.model_export_cloth:
            box2.prop(props, "model_export_cloth_attenuation")
            box2.prop(props, "model_export_cloth_mass")
            if model_type in {'MODEL', 'HEAD_GEAR', 'BODY_GEAR', 'RANGED'}:
                box2.prop(props, "model_export_cloth_general_folder")
            elif model_type == 'WEAPON':
                box2.prop(props, "model_export_cloth_weapon1_folder")
                box2.prop(props, "model_export_cloth_weapon2_folder")
            elif model_type == 'FOOT_GEAR':
                box2.prop(props, "model_export_cloth_foot1_folder")
                box2.prop(props, "model_export_cloth_foot2_folder")
        
        box3 = layout.box()
        box3.label(text="Childnode")

        box3.prop(props, "model_custom_childnode")
        if props.model_custom_childnode:
            if model_type != 'WEAPON':
                box3.prop(props, "macronode_vertex_group")
            elif model_type == 'WEAPON':
                box3.prop(props, "macronode_vertex_group_weapon_1")
                box3.prop(props, "macronode_vertex_group_weapon_2")
            box3.prop(props, "childnode_1_object")
            box3.prop(props, "childnode_2_object")
            box3.prop(props, "childnode_3_object")
            box3.prop(props, "childnode_4_object")

class VIEW3D_PT_gymnast_settings_object_settings(bpy.types.Panel):
    bl_label = "Object Alignment"
    bl_idname = "VIEW3D_PT_gymnast_settings_object_settings"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Gymnast Tool Suite'
    bl_parent_id = "VIEW3D_PT_gymnast_model_settings"
    bl_options = {'DEFAULT_CLOSED'}
    
    def draw(self, context):
        props = context.scene.gymnast_tool_model_props
        model_type = props.model_type_export
        layout = self.layout
        
        layout.prop(props, "model_is_advanced")
        if not props.model_is_advanced:
            layout.prop(props, "model_type_export")
        
        box = layout.box()
        
        if props.model_is_advanced or model_type == 'MODEL':
            box.label(text="Alignment")
            box.prop(props, "model_orientation")
            box.prop(props, "model_use_origin")
            if props.model_use_origin:
                box.prop(props, "model_origin_object")
            box.prop(props, "model_apply_constraint")
            
        elif model_type == 'WEAPON':
            box.label(text="Weapon Alignment")
            box.prop(props, "weapon_object_1")
            box.prop(props, "weapon_object_2")
            box.prop(props, "model_use_existing_object")
            
        elif model_type == 'FOOT_GEAR':
            box.label(text="Footwear Alignment")
            box.prop(props, "foot_object_1")
            box.prop(props, "foot_object_2")
            box.prop(props, "model_use_existing_object")
            
        elif model_type == 'HEAD_GEAR':
            box.label(text="Head Gear Alignment")
            box.prop(props, "selected_object")
            box.prop(props, "model_use_existing_object")
            box.prop(props, "model_align_flipped")
            
        elif model_type == 'BODY_GEAR':
            box.label(text="Body Gear Alignment")
            box.prop(props, "selected_object")
            box.prop(props, "model_body_top")
            box.prop(props, "model_body_middle")
            box.prop(props, "model_body_bottom")
            box.prop(props, "model_align_flipped")                
            
        elif model_type == 'RANGED':
            box.label(text="Ranged Alignment")
            box.prop(props, "selected_object")
            box.prop(props, "model_use_existing_object")
            
        box.operator("model.set_orientation", text="Set Alignment")
            
class VIEW3D_PT_gymnast_model_settings_misc(bpy.types.Panel):
    bl_label = "Miscellaneous"
    bl_idname = "VIEW3D_PT_gymnast_model_settings_misc"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Gymnast Tool Suite'
    bl_parent_id = "VIEW3D_PT_gymnast_model_settings"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        props = context.scene.gymnast_tool_model_props
        layout = self.layout
        box = layout.box()
        box.label(text="Offset")
        box.prop(props, "model_node_offset")
        box.prop(props, "model_edge_offset")
        box.prop(props, "model_tri_offset")

# Registration

classes = (
    ConvertXMLOperator,
    ExportModelToXML,
    AddNodesOperator,
    AddEdgesOperator,
    AddCapsulesOperator,
    SetOrientation,
    AddRuleOperator,
    RemoveRuleOperator,
    AddTemplateGroupsOperator,
    ClearMacroRulesOperator,
    GymnastToolModelSettings,
    MacroRuleItem,
    MACRO_UL_rules,
    VIEW3D_PT_gymnast_model_panel,
    VIEW3D_PT_gymnast_model_settings,
    VIEW3D_PT_gymnast_model_settings_import,
    VIEW3D_PT_gymnast_model_settings_export,
    VIEW3D_PT_gymnast_settings_object_settings,
    VIEW3D_PT_gymnast_model_settings_misc,
)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)
        
    bpy.types.Scene.gymnast_dependencies_xml = bpy.props.StringProperty(
        name="Dependencies XML",
        description="Select the dependencies/skeleton XML file",
        subtype="FILE_PATH"
    )
    bpy.types.Scene.gymnast_normal_xml = bpy.props.StringProperty(
        name="Model XML",
        description="Select the normal model XML file",
        subtype="FILE_PATH"
    )
    bpy.types.Scene.gymnast_tool_model_props = bpy.props.PointerProperty(type=GymnastToolModelSettings)
    bpy.types.Scene.macro_rules = bpy.props.CollectionProperty(type=MacroRuleItem)
    bpy.types.Scene.macro_rules_index = bpy.props.IntProperty()

def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
        
    del bpy.types.Scene.gymnast_dependencies_xml
    del bpy.types.Scene.gymnast_normal_xml
    del bpy.types.Scene.gymnast_tool_model_props
    del bpy.types.Scene.macro_rules
    del bpy.types.Scene.macro_rules_index

if __name__ == "__main__":
    register()