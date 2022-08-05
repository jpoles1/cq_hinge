import cadquery as cq
import math

def case(export_stl=True, show_board=False):

    ###
    #USER DEFINED VALUES
    ###

    board_w = 110 #width of PCB
    board_h = 84 #height of PCB
    box_inner_margin = 4 #padding space around PCB in box
    box_outer_fillet = 2
    
    board_thick = 30 #overall height of board
    box_height_split_lid_prop = 0.7; #Proportion of internal height in lid vs base
    floor_thick = 1 #box floor thickness
    ceil_thick = 2 #box ceiling thickness
    wall_thick = 2 #box wall thickness
    board_screw_w = 102 #horizontal screw spacing
    board_screw_h = 76 #vertical screw spacing 
    screw_diam = 3.6 #screw diameter
    hex_nut_rad = 6.2/2 #hex nut "radius" or dist across pts
    
    standoff_height = 4
    
    lid_lip_thick = 1.5 #thickness of lid inner lip
    lid_lip_height = 2 #height of lid inner lip
    lid_lip_clearance = 0.2 #space between lip outer wall and base inner wall
    
    arm_on_wall_length = 5
    arm_thickness = 2.5
    hook_ledge_depth = 2
    hook_height = 1.6
    hook_point_depth = 2
    hook_width = 8

    ###
    #COMPUTED VALUES
    ###

    box_iw = board_w + (box_inner_margin + lid_lip_thick) * 2
    box_ih = board_h + (box_inner_margin + lid_lip_thick) * 2
    base_height = (board_thick * (1-box_height_split_lid_prop)) + floor_thick + standoff_height
    lid_height = board_thick*box_height_split_lid_prop + ceil_thick
    
    def base():
        b = cq.Workplane("XY").box(box_iw+wall_thick*2, box_ih+wall_thick*2, base_height, centered=[1,1,0])
        b = b.edges("|Z").fillet(box_outer_fillet)
        b = b.faces(">Z").workplane().rect(box_iw, box_ih).cutBlind(-board_thick * (1-box_height_split_lid_prop) - standoff_height)
        b = b.faces("<Z[1]").workplane().rect(board_screw_w, board_screw_h).vertices().cylinder(standoff_height, (screw_diam + 3.5)/2, centered=[1,1,0])
        b = b.faces(">Z[2]").workplane().rect(board_screw_w, board_screw_h).vertices().hole(screw_diam)
        b = b.faces("<Z").rect(board_screw_w, board_screw_h).vertices().sketch().regularPolygon(hex_nut_rad ,6).finalize().cutBlind(3)
        return b
    def lid():
        l = cq.Workplane("XY").workplane(offset=base_height).box(box_iw+wall_thick*2, box_ih+wall_thick*2, lid_height, centered=[1,1,0])
        l = l.edges("|Z").fillet(box_outer_fillet)
        l = l.faces("XY").workplane().rect(box_iw-lid_lip_clearance*2, box_ih-lid_lip_clearance*2).extrude(-lid_lip_height)
        l = l.faces("<Z").workplane().rect(box_iw-lid_lip_thick*2, box_ih-lid_lip_thick*2).cutBlind(-(lid_lip_height + lid_height - ceil_thick))
        return l
    
    def lid_cutout():
        driver_cutout_h = 26
        l = lid()
        l = l.faces(">Z").workplane().move(0, board_h/2 - driver_cutout_h).rect(board_w + box_inner_margin, driver_cutout_h, centered=[1,0]).cutThruAll()
        return l
    
    def usb_cutout(orig):
        hole_h = 14
        hole_w = 16
        hole_dist_from_floor = standoff_height
        hole_dist_from_left = 20
        orig = orig.faces("<Y").workplane(origin=(0,0)).moveTo(-board_w/2 + hole_w/2 + hole_dist_from_left, floor_thick + hole_dist_from_floor + hole_h/2)\
            .sketch().rect(hole_w, hole_h).vertices().fillet(2).finalize().cutBlind(-(wall_thick + lid_lip_thick))
        return orig
    def power_cutout(orig):
        hole_h = 14
        hole_w = 16
        hole_dist_from_floor = 4
        hole_dist_from_bot = 10
        orig = orig.faces("<X").workplane(origin=(0,0)).moveTo(board_h/2 - hole_h/2 - hole_dist_from_bot, floor_thick + hole_dist_from_floor + hole_h/2)\
            .sketch().rect(hole_w, hole_h).vertices().fillet(2).finalize().cutBlind(-(wall_thick + lid_lip_thick))
        return orig
    def cutouts(orig):
        return power_cutout(usb_cutout(orig))

    def snap_clip():
        return cq.Sketch().polygon([
            [0,0],
            [0,arm_on_wall_length],
            [arm_thickness,arm_on_wall_length - (arm_thickness*math.tan(45 * math.pi / 180))],
            [arm_thickness,-hook_ledge_depth-hook_height],
            [0,-hook_ledge_depth-hook_height],
            [-hook_point_depth,-hook_ledge_depth-1],
            [-hook_point_depth,-hook_ledge_depth],
            [0.2,-hook_ledge_depth],
            [0.2, 0],
            [0,0]
        ])
        
    def lid_clips(l):
        def place_clip(face_sel_str):
            nonlocal l
            l = l.faces(face_sel_str).workplane(origin=cq.Vector(0,0,base_height-lid_lip_height))
            l = l.transformed(rotate=[0,90]).placeSketch(snap_clip()).extrude(hook_width/2, both=1)
        place_clip(">X[1]")
        place_clip("<X[1]")
        place_clip(">Y[1]")
        place_clip("<Y[1]")
        return l

    def base_clip_detent(b):
        def place_detent(face_sel_str):
            nonlocal b
            b = b.faces(face_sel_str).workplane(origin=(0,0,base_height-lid_lip_height-hook_ledge_depth-0.8))\
                .sketch().rect(hook_width+2, 2).finalize().cutBlind(-hook_point_depth)
        place_detent(">X[-2]")
        place_detent("<X[-2]")
        place_detent(">Y[-2]")
        place_detent("<Y[-2]")
        return b

    b = cutouts(base_clip_detent(base()))
    l = cutouts(lid_clips(lid_cutout()))

    display_lid_z_offset = 0
    a = cq.Assembly()\
        .add(b, loc =cq.Location(cq.Vector(0, 0, 0)), color=cq.Color(1,0,0,0.5), name="Base")\
        .add(l, loc =cq.Location(cq.Vector(0, 0, display_lid_z_offset)), color=cq.Color(0,0,1,0.5), name="Lid") 

    if(export_stl):
        cq.exporters.export(b, "stl/base.stl")
        cq.exporters.export(l, "stl/lid.stl")
        a.save("stl/case.step")

    if(show_board):
        pcb = cq.importers.importStep("../mks_gen_l_1.step")\
            .rotateAboutCenter((1,0,0), 90).translate((-7,-1.5, floor_thick+standoff_height+3)).rotateAboutCenter((0,0,1), 180)
        show_object(pcb)

    return a

c = case(export_stl=0, show_board=1)
show_object(c)
