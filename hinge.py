import cadquery as cq

def hinge(n_socket_arms = 3, socket_arm_w = 8, ball_arm_w = 3,
    socket_arm_l = 1, socket_arm_h = 2, hinge_base_l = 10,\
    socket_post_h = 5, socket_post_l = 5,\
    socket_ball_clearance = 0.15, interarm_clearance = 0.1, arm_base_chamfer = 0.5, arm_corner_fillet = 1,\
    folded_angle = 0, export_stl=0):

    ball_arm_l = socket_arm_l
    ball_arm_h = socket_arm_h
    ball_post_h = socket_post_h
    ball_post_l = socket_post_l

    ###
    #COMPUTED VALUES
    ###
    n_ball_arms = n_socket_arms + 1
    ball_diam = socket_post_l - 2

    ball_h = ball_arm_h + (ball_post_h-ball_arm_h) / 2 - 1
    socket_h = socket_arm_h + (socket_post_h-socket_arm_h) / 2 - 1

    ball_hinge_total_w = (n_socket_arms * socket_arm_w) + (n_ball_arms * ball_arm_w) + ((n_ball_arms + n_socket_arms - 1) * interarm_clearance)
    socket_hinge_total_w = ball_hinge_total_w - 2*(interarm_clearance+ball_arm_w)

    def socket_arm():
        s = cq.Workplane("XZ").sketch().polygon([
            [0, 0],
            [socket_arm_l+socket_post_l, 0],
            [socket_arm_l+socket_post_l, socket_arm_h],
            [socket_post_l, socket_arm_h],
            [socket_post_l, socket_post_h],
            [0, socket_post_h],
            [0, 0]
        ]).finalize().extrude(socket_arm_w/2, both=1)
        s = s.faces("<Z").edges("not |Y").chamfer(arm_base_chamfer)
        s = s.faces("<Z or >Z").edges("<X").fillet(arm_corner_fillet)
        def ball_cutout(face_sel_str):
            nonlocal s
            sphere_cutout = s.faces(face_sel_str)\
                .workplane(
                    offset=-socket_ball_clearance, 
                    origin=(socket_post_l / 2,0, socket_h)
                )\
                .sphere(socket_ball_clearance + (ball_diam)/2, combine=False)
            s = s.cut(sphere_cutout)
        ball_cutout("<Y")
        ball_cutout(">Y")
        return s
    
    #drop_ball allows you to skip drawing a ball on the positive (1) or negative (-1) side of the post
    def ball_arm(drop_ball=0):
        b = cq.Workplane("XZ").sketch().polygon([
            [0, 0],
            [ball_arm_l+ball_post_l, 0],
            [ball_arm_l+ball_post_l, ball_arm_h],
            [ball_post_l, ball_arm_h],
            [ball_post_l, ball_post_h],
            [0, ball_post_h],
            [0, 0]
        ]).finalize().extrude(ball_arm_w/2, both=1)
        b = b.faces("<Z").edges("not |Y").chamfer(arm_base_chamfer)
        b = b.faces("<Z or >Z").edges("<X").fillet(arm_corner_fillet)

        def ball(face_sel_str):
            nonlocal b
            b = b.faces(face_sel_str)\
                .workplane(
                    origin=(ball_post_l / 2,0, ball_h)
                )\
                .sphere(ball_diam/2)
        if drop_ball!=1:
            ball(">Y")
        if drop_ball!=-1:
            ball("<Y")

        return b

    def ball_hinge():    
        bh = cq.Workplane("XY").rect(hinge_base_l,ball_hinge_total_w,centered=[0,1,0]).extrude(2).translate((ball_arm_l+ball_post_l,0))
        start_y = -ball_hinge_total_w/2 + ball_arm_w/2
        spacing = ball_arm_w + socket_arm_w + 2*interarm_clearance

        for i in range(n_ball_arms):
            drop_ball = 0
            if i == 0:
                drop_ball = -1
            if i == n_ball_arms-1:
                drop_ball = 1
            
            bh = bh.workplane().union(ball_arm(drop_ball).translate([0,start_y+spacing*i]))

        return bh

    def socket_hinge():
        sh = cq.Workplane("XY").rect(hinge_base_l,ball_hinge_total_w,centered=[0,1,0]).extrude(2).translate((socket_arm_l+socket_post_l,0))
        start_y = -socket_hinge_total_w/2 + socket_arm_w/2
        spacing = ball_arm_w + socket_arm_w + 2*interarm_clearance

        for i in range(n_socket_arms):
            sh = sh.workplane().union(socket_arm().translate([0,start_y+spacing*i]))

        return sh

    bh = ball_hinge()
    sh = socket_hinge()

    alpha = 0.5
    a = cq.Assembly()
    if (folded_angle > 0 and not export_stl):
        a = a.add(
                ball_hinge().translate((-ball_post_l/2,0,-ball_h)),
                name="ball_hinge",
                loc=cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), -folded_angle),
                color=cq.Color(0,0.2,0,alpha)
            ).add(
                socket_hinge(), 
                name="socket_hinge",
                loc=cq.Location(cq.Vector(socket_post_l/2, 0, -socket_h), cq.Vector(0, 0, 1), 180),
                color=cq.Color(0,0,0.2,alpha)
            )
    else:
        a = a.add(
                ball_hinge(), 
                loc=cq.Location(cq.Vector(-ball_post_l/2, 0, 0)),
                color=cq.Color(0,0.2,0,alpha)
            ).add(
                socket_hinge(), 
                loc=cq.Location(cq.Vector(socket_post_l/2, 0, 0), cq.Vector(0, 0, 1), 180),
                color=cq.Color(0,0,0.2,alpha)
            )
    
    if(export_stl):
        cq.exporters.export(bh, "stl/ball_hinge.stl")
        cq.exporters.export(sh, "stl/socket_hinge.stl")
        cq.exporters.export(a.toCompound(), "stl/hinge.stl")
        #a.save("stl/hinge.step")
    
    a = a.toCompound()
    
    if (folded_angle > 0 and not export_stl):
        #May not work if socket + ball arm lengths are different?
        a = a.translate((ball_post_l/2 + socket_arm_l, 0, ball_h))
    else:
        a = a.translate((ball_post_l/2 + socket_arm_l, 0, 0))
    return a 

def fixed_width_hinge(hinge_w, n_socket_arms=3, arm_w_ratio=0.5, interarm_clearance=0.1, **args):
    n_ball_arms = n_socket_arms + 1
    socket_arm_w = (hinge_w - interarm_clearance * (n_socket_arms + n_ball_arms - 1)) / (n_ball_arms * arm_w_ratio + n_socket_arms)
    ball_arm_w = socket_arm_w * arm_w_ratio
    return hinge(n_socket_arms=n_socket_arms, socket_arm_w=socket_arm_w, ball_arm_w=ball_arm_w, **args)

#show_object(hinge(export_stl=0))
#show_object(fixed_width_hinge(40, folded_angle=90))