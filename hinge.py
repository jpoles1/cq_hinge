import cadquery as cq

default_opts = {
    "n_socket_arms": 3,
    "socket_arm_w": 8,
    "ball_arm_w": 3,
    "arm_l": 1,
    "arm_h": 2,
    "hinge_base_l": 10,
    "post_h": 5,
    "post_l": 5,
    "socket_ball_clearance": 0.15,
    "interarm_clearance": 0.1,
    "arm_base_chamfer": 0.5,
    "arm_corner_fillet": 1,
    "folded_angle": 0,
    "export_stl": 0
}

class Hinge:
    def __init__(self, **args):
        self.opts = default_opts
        self.reload_default_opts(**args)
        self.ball_socket_x = self.opts["post_l"] / 2
        self.ball_socket_z = self.opts["arm_h"] + (self.opts["post_h"]-self.opts["arm_h"]) / 2 - 1

    def reload_default_opts(self, **args):
        full_opts = self.opts.copy()
        for key, val in args.items():
            if key in full_opts:
                full_opts[key] = val
        self.opts = full_opts
        print(self.opts)
    def hinge(self):
        o = self.opts
        
        n_socket_arms = o["n_socket_arms"]
        socket_arm_w = o["socket_arm_w"]
        ball_arm_w = o["ball_arm_w"]
        arm_l = o["arm_l"]
        arm_h = o["arm_h"]
        post_l = o["post_l"]
        post_h = o["post_h"]
        hinge_base_l = o["hinge_base_l"]
        socket_ball_clearance = o["socket_ball_clearance"]
        interarm_clearance = o["interarm_clearance"]
        arm_base_chamfer = o["arm_base_chamfer"]
        arm_corner_fillet = o["arm_corner_fillet"]
        folded_angle = o["folded_angle"]
        export_stl = o["export_stl"]

        print(folded_angle)
        ###
        #COMPUTED VALUES
        ###
        n_ball_arms = n_socket_arms + 1
        ball_diam = post_l - 2

        ball_socket_x = self.ball_socket_x
        ball_socket_z = self.ball_socket_z

        ball_hinge_total_w = (n_socket_arms * socket_arm_w) + (n_ball_arms * ball_arm_w) + ((n_ball_arms + n_socket_arms - 1) * interarm_clearance)
        socket_hinge_total_w = ball_hinge_total_w - 2*(interarm_clearance+ball_arm_w)

        def socket_arm():
            s = cq.Workplane("XZ").sketch().polygon([
                [0, 0],
                [arm_l+post_l, 0],
                [arm_l+post_l, arm_h],
                [post_l, arm_h],
                [post_l, post_h],
                [0, post_h],
                [0, 0]
            ]).finalize().extrude(socket_arm_w/2, both=1)
            s = s.faces("<Z").edges("not |Y").chamfer(arm_base_chamfer)
            s = s.faces("<Z or >Z").edges("<X").fillet(arm_corner_fillet)
            def ball_cutout(face_sel_str):
                nonlocal s
                sphere_cutout = s.faces(face_sel_str)\
                    .workplane(
                        offset=-socket_ball_clearance, 
                        origin=(ball_socket_x, 0, ball_socket_z)
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
                [arm_l+post_l, 0],
                [arm_l+post_l, arm_h],
                [post_l, arm_h],
                [post_l, post_h],
                [0, post_h],
                [0, 0]
            ]).finalize().extrude(ball_arm_w/2, both=1)
            b = b.faces("<Z").edges("not |Y").chamfer(arm_base_chamfer)
            b = b.faces("<Z or >Z").edges("<X").fillet(arm_corner_fillet)

            def ball(face_sel_str):
                nonlocal b
                b = b.faces(face_sel_str)\
                    .workplane(
                        origin=(ball_socket_x,0, ball_socket_z)
                    )\
                    .sphere(ball_diam/2)
            if drop_ball!=1:
                ball(">Y")
            if drop_ball!=-1:
                ball("<Y")

            return b

        def ball_hinge():    
            bh = cq.Workplane("XY").rect(hinge_base_l,ball_hinge_total_w,centered=[0,1,0]).extrude(2).translate((arm_l+post_l,0))
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
            sh = cq.Workplane("XY").rect(hinge_base_l,ball_hinge_total_w,centered=[0,1,0]).extrude(2).translate((arm_l+post_l,0))
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
                    ball_hinge().translate((-ball_socket_x,0,-ball_socket_z)),
                    name="ball_hinge",
                    loc=cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), -folded_angle),
                    color=cq.Color(0,0.2,0,alpha)
                ).add(
                    socket_hinge(), 
                    name="socket_hinge",
                    loc=cq.Location(cq.Vector(ball_socket_x, 0, -ball_socket_z), cq.Vector(0, 0, 1), 180),
                    color=cq.Color(0,0,0.2,alpha)
                )
        else:
            a = a.add(
                    ball_hinge(), 
                    loc=cq.Location(cq.Vector(-ball_socket_x, 0, 0)),
                    color=cq.Color(0,0.2,0,alpha)
                ).add(
                    socket_hinge(), 
                    loc=cq.Location(cq.Vector(ball_socket_x, 0, 0), cq.Vector(0, 0, 1), 180),
                    color=cq.Color(0,0,0.2,alpha)
                )
        
        if(export_stl):
            cq.exporters.export(bh, "stl/ball_hinge.stl")
            cq.exporters.export(sh, "stl/socket_hinge.stl")
            cq.exporters.export(a.toCompound(), "stl/hinge.stl")
            #a.save("stl/hinge.step")
        
        a = a.toCompound()
        
        if (folded_angle > 0 and not export_stl): 
            a = a.translate((ball_socket_x + arm_l, 0, ball_socket_z))
        else:
            a = a.translate((ball_socket_x + arm_l, 0, 0))
        return a 

    def fixed_width_hinge(self, hinge_w, n_socket_arms=3, arm_w_ratio=0.5, interarm_clearance=0.1, **args):
        n_ball_arms = n_socket_arms + 1
        socket_arm_w = (hinge_w - interarm_clearance * (n_socket_arms + n_ball_arms - 1)) / (n_ball_arms * arm_w_ratio + n_socket_arms)
        ball_arm_w = socket_arm_w * arm_w_ratio
        self.reload_default_opts(n_socket_arms=n_socket_arms, socket_arm_w=socket_arm_w, ball_arm_w=ball_arm_w, **args)
        return self.hinge()

#show_object(Hinge(export_stl=0).hinge())
#show_object(Hinge(folded_angle=90).fixed_width_hinge(40))