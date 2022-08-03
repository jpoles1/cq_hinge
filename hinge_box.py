import cadquery as cq
from hinge import Hinge

box_iw = 30
box_il = 40
box_ih = 20
wall_thick = 2
floor_thick = 3

box_ow = box_iw + wall_thick * 2 
box_ol = box_il + wall_thick * 2
box_oh = box_ih + floor_thick

def hinge_box(wall_angle = 0, ceil_angle = 0, export_stl=0):
    def bbox_solid(shape, min_offset=(0,0,0), max_offset=(0,0,0)):
        bbox = shape.BoundingBox()
        return cq.Solid.makeBox(
            bbox.xlen+min_offset[0]+max_offset[0],
            bbox.ylen+min_offset[1]+max_offset[1],
            bbox.zlen+min_offset[2]+max_offset[2],
            pnt=cq.Vector(bbox.xmin-min_offset[0], bbox.ymin-min_offset[1], bbox.zmin-min_offset[2]),
        )
    def hinge_bbox_solid(h):
        return bbox_solid(h, (0, 0.5, 0), (0, 0.5, 1))

    #Draw box outer contour
    b = cq.Workplane("XY").box(box_ow, box_ol, box_oh, centered=[1,1,0])
    #Hollow out box
    b = b.faces(">Z").workplane().move(wall_thick, 0).rect(box_iw + wall_thick * 2, box_il).cutBlind(-box_ih)
    #Cut out notch for lid to sit on when folded up
    b = b.faces("<X").edges(">Z").workplane(centerOption="CenterOfMass",invert=1).rect(box_il, wall_thick+0.5, centered=[1,0]).cutBlind(wall_thick)




    #Create and position wall hinge
    unfold_wallHinge = Hinge()
    wallHinge = Hinge(folded_angle=wall_angle)
    wall_hinge = wallHinge.fixed_width_hinge(box_il-1).translate((box_ow/2 - wallHinge.ball_socket_x,0,0))
    unfold_wall_hinge = unfold_wallHinge.fixed_width_hinge(box_il-1).translate((box_ow/2 - wallHinge.ball_socket_x,0,0))
    #Create and position ceiling hinge
    unfold_ceilHinge = Hinge()
    ceilHinge = Hinge(folded_angle=ceil_angle)
    ceil_hinge = ceilHinge.fixed_width_hinge(box_il-1).translate((box_ow/2 + box_oh - 3 * ceilHinge.ball_socket_x, 0, 0))
    unfold_ceil_hinge = unfold_ceilHinge.fixed_width_hinge(box_il-1).translate((box_ow/2 + box_oh - 3 * ceilHinge.ball_socket_x, 0, 0))

    #Create wall (will lie flat on XY initially)
    hinged_wall = b.faces(">X").workplane(origin=(0,0,0)).box(box_il-1, wall_thick, box_oh-wallHinge.total_l, centered=[1, 0, 0], combine=False)
    #Create ceiling (will lie flat on XY initially next to wall)
    hinged_ceil = hinged_wall.faces(">X").workplane().box(box_il-1, wall_thick, box_ow-4, centered=[1, 0, 0], combine=False)
    #Cut hinge out of ceiling
    hinged_ceil = hinged_ceil.cut(bbox_solid(unfold_ceil_hinge))
    #Rotate ceiling around it's physical axis of rotation
    hinged_ceil = hinged_ceil.rotate((box_ow/2 + box_oh - 3 * ceilHinge.ball_socket_x, 0, ceilHinge.ball_socket_z), (box_ow/2 + box_oh - 3 * ceilHinge.ball_socket_x, 1, ceilHinge.ball_socket_z), -ceil_angle)
    #Combine wall and ceiling w/ ceiling hinge 
    hinged_wall = hinged_wall.union(hinged_ceil).cut(bbox_solid(unfold_ceil_hinge)).union(ceil_hinge)
    hinged_wall = hinged_wall.cut(bbox_solid(unfold_wall_hinge))
    #Rotate wall around it's physical axis of rotation
    hinged_wall = hinged_wall.rotate((box_ow/2  - wallHinge.ball_socket_x, 0, wallHinge.ball_socket_z), (box_ow/2 - wallHinge.ball_socket_x, 1, wallHinge.ball_socket_z), -wall_angle) 

    #Cutout clearance for hinge on box before adding hinged wall
    b = b.faces(">X").cut(hinge_bbox_solid(wall_hinge))
    #Combine base box and hinged wall + ceiling w/ ceiling hinge 
    b = b.union(hinged_wall).union(wall_hinge)

    if(export_stl):
        cq.exporters.export(b, "stl/hinged_box.stl")

    return b

show_object(hinge_box(export_stl=1))
show_object(hinge_box(45, 45).translate((0, box_il * 1.5, 0)))
show_object(hinge_box(90, 90).translate((0, box_il * 3, 0)))
