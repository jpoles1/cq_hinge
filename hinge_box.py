import cadquery as cq
from hinge import Hinge

default_opts = {
    "box_iw": 25,
    "box_il": 35,
    "box_ih": 25,
    "wall_thick": 2,
    "standoff_h": 6,
}

def bbox_solid(shape, min_offset=(0,0,0), max_offset=(0,0,0)):
        bbox = shape.BoundingBox()
        return cq.Solid.makeBox(
            bbox.xlen+min_offset[0]+max_offset[0],
            bbox.ylen+min_offset[1]+max_offset[1],
            bbox.zlen+min_offset[2]+max_offset[2],
            pnt=cq.Vector(bbox.xmin-min_offset[0], bbox.ymin-min_offset[1], bbox.zmin-min_offset[2]),
        )

class HingeBox:
    def __init__(self, **args):
        self.opts = default_opts
        self.reload_default_opts(**args)

    def reload_default_opts(self, **args):
        full_opts = self.opts.copy()
        for key, val in args.items():
            if key in full_opts:
                full_opts[key] = val
        self.opts = full_opts
        #Recompute values for publically avail properties
        self.box_ow =  self.opts["box_iw"] + self.opts["wall_thick"] * 2 
        self.box_ol = self.opts["box_il"] + self.opts["wall_thick"] * 2
        self.box_oh = self.opts["box_ih"] + self.opts["wall_thick"]

    def add_standoffs(self, b, pts, screw_diam=3.6, hex_nut_rad=6.2/2):
        for pt in pts:
            b = b.cut(cq.Workplane("XY", origin=(pt[0], pt[1], 0)).sketch().regularPolygon(hex_nut_rad, 6).finalize().extrude(3))
            b = b.union(cq.Workplane("XY", origin=(pt[0], pt[1], self.opts["wall_thick"])).circle(2+screw_diam/2).circle(screw_diam/2).extrude(self.opts["standoff_h"]))
        return b

    def lid_cutout():
        driver_cutout_h = 26
        l = lid()
        l = l.faces(">Z").workplane().move(0, board_l/2 - driver_cutout_h).rect(board_w + box_inner_margin, driver_cutout_h, centered=[1,0]).cutThruAll()
        return l

    def hinge_box(self, wall_angle = 0, ceil_angle = 0, screw_closure = 1, standoffs=[], export_stl=0, wall_cutouts=None, top_cutouts=None):
        o = self.opts

        box_iw = o["box_iw"]
        box_il = o["box_il"]
        box_ih = o["box_ih"]
        wall_thick = o["wall_thick"]

        box_ow = self.box_ow
        box_ol = self.box_ol
        box_oh = self.box_oh
        
        #Creates a bounding box for a hinge with some margins for clearance
        def hinge_margin_bbox_solid(h):
            return bbox_solid(h, (0, 0.5, 0), (0, 0.5, 1))

        #Draw box outer contour
        b = cq.Workplane("XY").box(box_ow, box_ol, box_oh, centered=[1,1,0])
        #Hollow out box
        b = b.faces(">Z").workplane().move(wall_thick, 0).rect(box_iw + wall_thick * 2, box_il).cutBlind(-box_ih)
        #Cut out notch for lid to sit on when folded up
        b = b.faces("<X").edges(">Z").workplane(centerOption="CenterOfMass",invert=1).rect(box_il, wall_thick+0.5, centered=[1,0]).cutBlind(wall_thick)
        
        if wall_cutouts:
            b = wall_cutouts(self, b)

        if len(standoffs) > 0:
            b = self.add_standoffs(b, standoffs)

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

        hinge_blocker_w = 3
        hinge_blocker_l = 7
        hinge_blocker_h = 6

        #Create wall (will lie flat on XY initially)
        hinged_wall = b.faces(">X").workplane(origin=(0,0,0)).box(box_il-1, wall_thick, box_oh-wallHinge.total_l, centered=[1, 0, 0], combine=False)
        #Create ceiling (will lie flat on XY initially next to wall)
        hinged_ceil = hinged_wall.faces(">X").workplane().box(box_il-1, wall_thick, box_ow-ceilHinge.total_l + ceilHinge.ball_socket_x, centered=[1, 0, 0], combine=False)
        #Cut hinge out of ceiling
        hinged_ceil = hinged_ceil.cut(bbox_solid(unfold_ceil_hinge))
        
        if screw_closure:
            latch_w = 8
            latch_l = 10
            latch_h = 10
            nut_r = 6.2/2
            nut_h = 3
            screw_diam = 3.6
            #Add ceiling nut holder for case screw latch
            latch = cq.Workplane("XY").box(latch_w, latch_l, latch_h, centered=[0, 1, 0])  
            latch = latch.faces(">X").workplane(origin=(0, 0, latch_h/2)).circle(screw_diam/2).cutBlind(-latch_w)
            latch = latch.faces(">X").workplane(origin=(0, 0, latch_h/2), offset=-latch_w).sketch().regularPolygon(nut_r, 6).finalize().cutBlind(nut_h)
            hinged_ceil = hinged_ceil.union(latch.translate((3*box_ow/2 + box_oh - 2*ceilHinge.total_l + ceilHinge.ball_socket_x - wall_thick - latch_w - 1, 0, wall_thick)))
            #Cut latch hole
            b = b.faces("<X").workplane(origin=(0, 0, box_oh - wall_thick - latch_h / 2 + 1)).circle(screw_diam/2).cutBlind(-wall_thick)

        if top_cutouts:
            hinged_ceil = top_cutouts(self, hinged_ceil)
        
        #Rotate ceiling around it's physical axis of rotation
        hinged_ceil = hinged_ceil.rotate((box_ow/2 + box_oh - 3 * ceilHinge.ball_socket_x, 0, ceilHinge.ball_socket_z), (box_ow/2 + box_oh - 3 * ceilHinge.ball_socket_x, 1, ceilHinge.ball_socket_z), -ceil_angle)
        #Combine wall and ceiling w/ ceiling hinge 
        hinged_wall = hinged_wall.union(hinged_ceil).cut(bbox_solid(unfold_ceil_hinge)).union(ceil_hinge)
        hinged_wall = hinged_wall.cut(bbox_solid(unfold_wall_hinge))
        #Add hinge blocker for ceiling
        ceil_hinge_block = cq.Workplane("XY", origin=(box_ow/2+box_oh-ceilHinge.total_l-wall_thick-hinge_blocker_w, 0, wall_thick)).box(hinge_blocker_w,hinge_blocker_l,hinge_blocker_h, centered=[0, 1, 0])
        hinged_wall = hinged_wall.union(ceil_hinge_block)
        #Rotate wall around it's physical axis of rotation
        hinged_wall = hinged_wall.rotate((box_ow/2  - wallHinge.ball_socket_x, 0, wallHinge.ball_socket_z), (box_ow/2 - wallHinge.ball_socket_x, 1, wallHinge.ball_socket_z), -wall_angle) 

        #Cutout clearance for hinge on box before adding hinged wall
        b = b.faces(">X").cut(hinge_margin_bbox_solid(wall_hinge))
        #Combine base box and hinged wall + ceiling w/ ceiling hinge 
        b = b.union(hinged_wall).union(wall_hinge)
        #Create wall hinge blockers
        wall_hinge_block = cq.Workplane("XY", origin=(box_ow/2-wall_thick-hinge_blocker_w, 0, wall_thick)).box(hinge_blocker_w,hinge_blocker_l,hinge_blocker_h, centered=[0, 1, 0])
        b = b.union(wall_hinge_block)

        if(export_stl):
            cq.exporters.export(b, "stl/hinged_box.stl")

        return b
    def demo(self):  
        show_object(self.hinge_box(export_stl=0))
        show_object(self.hinge_box(90, 90).translate((0, self.opts["box_il"] * 1.5, 0)))
        #show_object(self.hinge_box(90, 90).translate((0, self.opts["box_il"] * 3, 0)))
        #show_object(self.hinge_box(45, 45).translate((0, box_il * 1.5, 0)))
        #show_object(self.hinge_box(90, 90).translate((0, box_il * 3, 0)))
        #show_object(cq.Assembly().add(self.hinge_box(90, 90).translate((0, box_il * 3, 0)), color=cq.Color(0,1,0,0.25)))

#HingeBox().demo()
#self.hinge_box(export_stl=1)
'''
board_w = 84 #width of PCB
board_l = 110 #length/depth of PCB
box_inner_margin = 12 #padding space around PCB in box

box_iw = board_w + 2*box_inner_margin
box_il = board_l + 2*box_inner_margin

board_screw_w = 76 #horizontal screw spacing
board_screw_l = 102 #vertical screw spacing 
screw_diam = 3.6 #screw diameter
hex_nut_rad = 6.2/2 #hex nut "radius" or dist across pts

x = HingeBox(box_iw=box_iw, box_il = box_il, box_id = 30).hinge_box(standoffs=[
    (board_screw_w / 2, board_screw_l / 2),
    (-board_screw_w / 2, board_screw_l / 2),
    (-board_screw_w / 2, -board_screw_l / 2),
    (board_screw_w / 2, -board_screw_l / 2),
])
'''