# export
#     SceneStructureOverlay,
#     SubSceneExtractParamsOverlay,
#     StructureFollowCamera

# function AutoViz.render!{D,I}(rendermodel::RenderModel, structure::SceneStructure, scene::EntityFrame{PosSpeed1D, D, I};
#     line_width = 0.25, # m
#     circle_radius = 0.5, # m
#     color_line::Colorant = AutoViz.COLOR_ASPHALT,
#     factor_colors::Tuple{Colorant,Colorant,Colorant} =
#         (RGBA(0x52/0xFF,0xE3/0xFF,0xF6/0xFF),
#          RGBA(0xA7/0xFF,0xEC/0xFF,0x21/0xFF),
#          RGBA(0xFF/0xFF,0x00/0xFF,0x7F/0xFF))
#     )

#     # render all links first
#     for fa in structure.factor_assignments
#         if length(fa.vehicle_indeces) == 2
#             # draw a line between the vehicle centers

#             pos1 = get_center(scene.vehicles[fa.vehicle_indeces[1]])
#             pos2 = get_center(scene.vehicles[fa.vehicle_indeces[2]])

#             add_instruction!(rendermodel, render_line_segment, (pos1.x, pos1.y, pos2.x, pos2.y, color_line, line_width))
#         end
#     end

#     for fa in structure.factor_assignments
#         if length(fa.vehicle_indeces) == 1
#             # draw a line from the vehicle center to the bottom right

#             veh = get_center(scene.vehicles[fa.vehicle_indeces[1]])
#             color = factor_colors[fa.form]

#             add_instruction!(rendermodel, render_circle, (pos.x, pos.y, circle_radius, color))
#         elseif length(fa.vehicle_indeces) == 2
#             # draw a line between the vehicle centers

#             pos1 = get_center(scene.vehicles[fa.vehicle_indeces[1]])
#             pos2 = get_center(scene.vehicles[fa.vehicle_indeces[2]])
#             center = lerp(pos1, pos2, 0.5)
#             color = factor_colors[fa.form]

#             add_instruction!(rendermodel, render_circle, (center.x, center.y, circle_radius, color))
#         else
#             error("not implemented")
#         end
#     end

#     rendermodel
# end

# type SceneStructureOverlay <: SceneOverlay
#     structure::Union{Void,SceneStructure}
# end
# function AutoViz.render!(rendermodel::RenderModel, overlay::SceneStructureOverlay, scene::Scene, roadway::Roadway)
#     if isa(overlay.structure, SceneStructure)
#         structure = overlay.structure
#     else
#         structure = gen_scene_structure(scene, roadway, create_shared_factors())
#     end

#     render!(rendermodel, structure, scene)
# end

# type SubSceneExtractParamsOverlay <: SceneOverlay
#     scene_extract::SubSceneExtractParams
#     color::Colorant
#     line_width::Float64

#     function SubSceneExtractParamsOverlay(
#         scene_extract::SubSceneExtractParams;
#         color::Colorant=colorant"red",
#         line_width::Float64=1.0,
#         )

#         new(scene_extract, color, line_width)
#     end
# end
# function AutoViz.render!(rendermodel::RenderModel, poly::ConvexPolygon, color::Colorant, line_width::Real)

#     pts = Array(Float64, 2, length(poly))
#     for (i,p) in enumerate(poly)
#         pts[1,i] = p.x
#         pts[2,i] = p.y
#     end

#     add_instruction!(rendermodel, render_closed_line, (pts, color, line_width))

#     rendermodel
# end
# AutoViz.render!(rendermodel::RenderModel, overlay::SubSceneExtractParamsOverlay, scene::Scene, roadway::Roadway) = render!(rendermodel, overlay.scene_extract.box, overlay.color, overlay.line_width)

# type StructureFollowCamera <: Camera
#     structure::SceneStructure
#     zoom::Float64 # [pix/meter]
#     StructureFollowCamera(structure::SceneStructure, zoom::Float64=3.0) = new(structure, zoom)
# end
# function AutoViz.camera_set!(rendermodel::RenderModel, cam::StructureFollowCamera, scene::Scene, roadway::Roadway, canvas_width::Int, canvas_height::Int)


#     if length(cam.structure.active_vehicles) > 0

#         # get camera center
#         C = VecE2(0.0,0.0)
#         for vehicle_index in cam.structure.active_vehicles
#             veh = scene[vehicle_index]
#             C += convert(VecE2, veh.state.posG)
#         end
#         C = C / length(cam.structure.active_vehicles)

#         camera_set_pos!(rendermodel, C)
#         camera_setzoom!(rendermodel, cam.zoom)
#     else
#         add_instruction!( rendermodel, render_text, ("StructureFollowCamera did not find any vehicles", 10, 15, 15, colorant"white"), incameraframe=false)
#         camera_fit_to_content!(rendermodel, canvas_width, canvas_height, 0.1)
#     end

#     rendermodel
# end
