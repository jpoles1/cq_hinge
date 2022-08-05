import cadquery as cq
from hinge_box import HingeBox

board_w = 84 #width of PCB
board_l = 110 #height of PCB
box_inner_margin = 6 #padding space around PCB in box
wall_thick = 1.5

box_iw = board_w + 2*box_inner_margin
box_il = board_l + 2*box_inner_margin
box_ih = 25

board_screw_w = 76 #horizontal screw spacing
board_screw_l = 102 #vertical screw spacing 
screw_diam = 3.6 #screw diameter
hex_nut_rad = 6.2/2 #hex nut "radius" or dist across pts
standoff_h = 4

def usb_cutout(self, orig):
    hole_h = 14
    hole_w = 16
    hole_dist_from_floor = self.opts["standoff_h"]
    hole_dist_from_left = 20
    orig = orig.faces("<X").workplane(origin=(0,0)).moveTo(-board_l/2 + hole_w/2 + hole_dist_from_left, self.opts["wall_thick"] + hole_dist_from_floor + hole_h/2)\
        .sketch().rect(hole_w, hole_h).vertices().fillet(2).finalize().cutBlind((-self.opts["wall_thick"]))
    return orig

def power_cutout(self, orig):
    hole_h = 14
    hole_w = 16
    hole_dist_from_floor = 4
    hole_dist_from_bot = 1
    orig = orig.faces(">Y").workplane(origin=(0,0)).moveTo(board_w/2 - hole_h - hole_dist_from_bot, self.opts["wall_thick"] + hole_dist_from_floor + hole_h/2)\
        .sketch().rect(hole_w, hole_h).vertices().fillet(2).finalize().cutBlind((-self.opts["wall_thick"]))
    return orig

def top_cutout(self, orig):
    driver_cutout_w = 28
    orig = orig.faces("<Z").workplane(origin=(self.box_ow/2 + self.box_oh + 4, 0, 0)).rect(driver_cutout_w, board_l, centered=[0,1]).cutThruAll()
    return orig

def cutouts(self, orig):
    return top_cutout(self, power_cutout(self, usb_cutout(self, orig)))

b = HingeBox(box_iw=box_iw, box_il = box_il, box_ih = box_ih, wall_thick=wall_thick, standoff_h=standoff_h).hinge_box(wall_angle=0, ceil_angle=0, standoffs=[
    (board_screw_w / 2, board_screw_l / 2),
    (-board_screw_w / 2, board_screw_l / 2),
    (-board_screw_w / 2, -board_screw_l / 2),
    (board_screw_w / 2, -board_screw_l / 2),
], wall_cutouts=cutouts, top_cutouts=top_cutout)

show_object(b)
cq.exporters.export(b, "stl/mks_hinged_box.stl") 

show_board=0
if(show_board):
    pcb = cq.importers.importStep("mks_gen_l_1.step")\
        .rotateAboutCenter((1,0,0), 90).translate((-2,1.5, wall_thick+standoff_h+3)).rotateAboutCenter((0,0,1), 90)
    show_object(pcb)
