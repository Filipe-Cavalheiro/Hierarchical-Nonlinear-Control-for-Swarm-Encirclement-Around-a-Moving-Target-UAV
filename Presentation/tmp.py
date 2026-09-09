class APF(ThreeDSlide):
    def construct(self):
        Title = Text("5. APF", font_size=28).to_corner(UL)
        self.add_fixed_in_frame_mobjects(Title)
        # ============================================================
        # Helper: create the split-screen layout
        # ============================================================
        dot = Dot3D()

        divider = Line(
            start=UP * 3.8,
            end=DOWN * 3.8,
            stroke_width=2,
        ).shift(RIGHT * 2.0)
        
        screen_width = config.frame_width
        line_x = divider.get_x()

        active_title = Tex(
            r"\textbf{Equations}",
            font_size=30,
            color=YELLOW
        )

        active_title_middle_x = (line_x + screen_width / 2) / 2
        active_title.set_x(active_title_middle_x)  
        active_title.shift(UP * 3.1)
        self.add_fixed_in_frame_mobjects(divider, active_title)

        # ============================================================
        # Axes
        # ============================================================

        left = np.array([
            np.sin(CAMERA_THETA),
            -np.cos(CAMERA_THETA),
            0
        ])

        axes = ThreeDAxes()

        x_label = axes.get_x_axis_label(Tex("x"))
        y_label = axes.get_y_axis_label(Tex("y")).shift(UP * 1.8)
        z_label = axes.get_z_axis_label(Tex("z")).rotate_about_origin(PI)

        axes_group = VGroup(
            axes,
            x_label,
            y_label,
            z_label
        )

        axes_group.shift(left*2.5)
        AXIS_ORIGIN = axes.c2p(0, 0, 0)

        self.set_camera_orientation(
            phi=CAMERA_PHI,
            theta=CAMERA_THETA,
            zoom=1
        )

        self.add(axes_group)

        # ============================================================
        # Architecture
        # ============================================================
        geometric = box("Geometric\nController")
        apf = box("Safe Guard\n(APF)")
        radial = box("Radii\nControl")
        nonlinear = box("Nonlinear\nController")
        uav = box("UAV\nModel")

        # Centered positions
        geometric.move_to(LEFT * 3 + UP * 1.5)
        radial.move_to(RIGHT * 1.0 + UP * 1.5)

        apf.move_to(LEFT * 3 + DOWN * 0.5)
        nonlinear.move_to(RIGHT * 1.0 + DOWN * 0.5)
        uav.move_to(RIGHT * 5.0 + DOWN * 0.5)

        # Shift everything together to guarantee centering
        architecture_boxes = VGroup(
            geometric, apf, radial, nonlinear, uav
        )
        
        # ------------------ Arrows ------------------
        a_input_p = Arrow(
            geometric.get_left() - np.array([0.5, 0.4, 0]),
            geometric.get_left() - np.array([0.0, 0.4, 0]),
            buff=0.1,
        )

        a_input_p_star = Arrow(
            geometric.get_left() - np.array([0.5, -0.4, 0]),
            geometric.get_left() - np.array([0.0, -0.4, 0]),
            buff=0.1,
        )   

        a1 = Arrow(geometric.get_bottom(), apf.get_top(), buff=0.1)
        a2 = Arrow(apf.get_right(), nonlinear.get_left(), buff=0.1)
        a3 = Arrow(radial.get_bottom(), nonlinear.get_top(), buff=0.1)

        a_torque = Arrow(
            nonlinear.get_right() + np.array([0.0, 0.4, 0]),
            uav.get_left() + np.array([0.0, 0.4, 0]), 
            buff=0.1
        )

        a_force = Arrow(
            nonlinear.get_right() - np.array([0.0, 0.4, 0]),
            uav.get_left() - np.array([0.0, 0.4, 0]), 
            buff=0.1
        )  

        a_output_p = Arrow(
                    uav.get_top(),
                    uav.get_top() + np.array([0.0, 1.5, 0]),
                    buff=0.1,
        )     

        architecture_arrows = VGroup(
            a1, 
            a2,
            a3,
            a_torque,
            a_force,
            a_input_p,
            a_input_p_star,
            a_output_p
        )

        # ------------------ Labels ------------------
        label_font_size = 28
        label_p_input = MathTex(
            "\Pi_{\Gamma^2}^{1,0}(p)", font_size=label_font_size
        ).next_to(a_input_p, LEFT, buff=0.1)

        label_p_star = MathTex(
            "\Pi_{\Gamma^2}^{1,0}(p^\star)", font_size=label_font_size
        ).next_to(a_input_p_star, LEFT, buff=0.1)

        label_p_output = Text(
            "p", font_size=label_font_size
        ).next_to(a_output_p, LEFT, buff=0.1)

        architecture_labels = VGroup(
            label_p_input,
            label_p_star,
            MathTex("q_i", font_size=label_font_size).next_to(a1, RIGHT, buff=0.1),
            MathTex("r_i", font_size=label_font_size).next_to(a3, RIGHT, buff=0.1),
            Text("τ", font_size=label_font_size).next_to(a_torque, UP, buff=0.1),
            MathTex("f", font_size=label_font_size).next_to(a_force, UP, buff=0.1),
            label_p_output
        )

        architecture = VGroup(
            architecture_boxes,
            architecture_arrows,
            architecture_labels
        )
        architecture.scale(0.5)
        self.add_fixed_in_frame_mobjects(architecture)
        architecture.to_corner(DL, buff=0.3)

        # one dot for each direction
        target_uav = dot.copy().set_color(RED).shift(OUT + AXIS_ORIGIN + RIGHT - LEFT)
        target_uav_position = OUT + RIGHT - LEFT
        agents_list = []
        for agent_pos in agents_UAV_pos:
            agent_uav = (
                dot.copy()
                .set_color(WHITE)
                .move_to(agent_pos + target_uav.get_center())
            )
            agents_list.append(agent_uav)

        indicate_scale = 1.4
        self.play(
            FadeIn(Title),
            apf.animate.scale(indicate_scale),
            FadeIn(target_uav),
            FadeIn(*agents_list)
        )   

        proj_eq = MathTex(
            r"\Pi_{\Gamma^2}^{1,p_T}(p_i) = \frac{p-p_T}{\|p-p_T\|}",
            font_size=28
        ).next_to(active_title, DOWN, buff=0.18)
        self.add_fixed_in_frame_mobjects(proj_eq)

        animation = []

        for agent in agents_list:
            current_pos = agent.get_center() - AXIS_ORIGIN - target_uav_position

            # Project onto unit sphere centered at origin
            target_pos = current_pos / np.linalg.norm(current_pos)

            # Move from current position to projected position
            animation.append(
                agent.animate.move_to(target_pos + AXIS_ORIGIN + target_uav_position)
            )

        # ============================================================
        # 3D SPHERE
        # ============================================================
        
        R = 1.0
        sphere = Sphere(
            radius=R,
            resolution=(32, 64),
            fill_opacity=0.10,
            stroke_opacity=0.35,
        ).move_to(AXIS_ORIGIN + target_uav_position)

        # Equatorial reference circle
        equator = Circle(
            radius=R,
            stroke_opacity=0.22,
        ).rotate(PI / 2, axis=RIGHT).move_to(AXIS_ORIGIN + target_uav_position)

        self.next_slide()

        self.play(
            FadeIn(proj_eq),
            *animation,
            FadeIn(sphere),
            Create(equator),
        )

        # Fixed labels so they remain readable
        label_i = Tex(r"$q_i$", font_size=27, color=RED)
        label_i_point = self.camera.project_point(q_i)
        label_i.move_to(label_i_point + UP * 0.35)
        self.add_fixed_in_frame_mobjects(label_i)

        label_j = Tex(r"$q_j$", font_size=27, color=BLUE)
        label_j_point = self.camera.project_point(q_j)
        label_j.move_to(label_j_point + UP * 0.35)
        self.add_fixed_in_frame_mobjects(label_j)

        self.next_slide()
        self.play(
            FadeOut(target_uav),
            uav_i.animate.set_color(RED),
            uav_j.animate.set_color(BLUE),
            FadeIn(label_i), 
            FadeIn(label_j)
        )
