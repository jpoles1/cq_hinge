import cadquery as cq
from hinge import hinge, fixed_width_hinge

box_iw = 80
box_il = 100
box_ih = 30
wall_thick = 2
floor_thick = 3

box_ow = box_iw + wall_thick * 2 
box_ol = box_il + wall_thick * 2
box_oh = box_ih + floor_thick

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

b = cq.Workplane("XY").box(box_ow, box_ol, box_oh, centered=[1,1,0])
b = b.faces(">Z").workplane().move(wall_thick, 0).rect(box_iw + wall_thick, box_il).cutBlind(-box_ih)
#b = b.faces(">Z").workplane().rect(box_iw, box_il).cutBlind(-box_ih)

wall_angle = 0
h = fixed_width_hinge(box_il, folded_angle=wall_angle, hinge_base_l=0.1).translate((box_ow/2 - 6 + 0.,0,0))
hinged_wall = b.faces(">X").workplane(origin=(0,0,0), offset=0.1).box(box_il-1, wall_thick, box_oh, centered=[1, 0, 0], combine=False)
hinged_wall = hinged_wall.rotate((box_ow/2, 0, 0), (box_ow/2, 1, 0), -wall_angle)

b = b.faces(">X")
#Cutout clearance for hinge on box before adding hinged wall
b = b.cut(hinge_bbox_solid(h))
b = b.union(hinged_wall).cut(bbox_solid(h)).union(h)


show_object(b)


 
