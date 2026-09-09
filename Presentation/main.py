from manim import *
from manim_slides import Slide, ThreeDSlide
from helper_funcs import *

CAMERA_THETA = 30 * DEGREES
CAMERA_PHI = 75 * DEGREES
agents_UAV_pos = [[-0.3, 1.0, 0.1], [0.3, -1.0, -0.2], [1.2, -0.8, 2]]

class Presentation(ThreeDSlide):
    def construct(self):
        self.title_card()
        self.intro()        
        self.objective()

    def title_card(self):
        title = VGroup(
            Text("Hierarchical Nonlinear Control for Swarm", font_size=48),
            Text("Encirclement Around a Moving Target UAV", font_size=48),
            Text(
                "Filipe Cavalheiro and Bruno J. Guerreiro",
                font_size=24,
                t2c={
                    "Filipe Cavalheiro": "YELLOW",
                    "Bruno J. Guerreiro": "BLUE",
                },
            )
        ).arrange(DOWN)

        fct_logo = ImageMobject("assets/images/logo_nova_fct_pt_v.png")
        fct_logo.set(width=2)
        fct_logo.to_corner(UL, buff = 0.6)

        self.add(fct_logo)
        self.play(FadeIn(title))

        self.next_slide()
        self.wipe(Group(title, fct_logo))

    def intro(self):
        axes = ThreeDAxes()

        x_label = axes.get_x_axis_label(Tex("x"))
        y_label = axes.get_y_axis_label(Tex("y")).shift(UP * 1.8)
        z_label = axes.get_z_axis_label(Tex("z")).rotate_about_origin(PI)

        # 3D variant of the Dot() object
        dot = Dot3D()

        # zoom out so we see the axes
        self.set_camera_orientation(zoom=0.5)

        self.play(FadeIn(axes), FadeIn(x_label), FadeIn(y_label), FadeIn(z_label))

        self.wait(0.5)

        # animate the move of the camera to properly see the axes
        self.move_camera(phi=CAMERA_PHI, theta=CAMERA_THETA, zoom=1, run_time=1.5)

        # one dot for each direction
        target_uav = dot.copy().set_color(RED)

        self.play(
            target_uav.animate.shift(OUT)
        )

        # -------------------------
        # Screen-space legend
        # -------------------------

        # Target UAV legend
        target_legend_dot = Dot(
            radius=0.08,
            color=RED,
        )

        target_legend_text = MathTex(
            r"\text{target UAV }(p_T)",
            color=RED,
            font_size=36,
        )
        
        target_legend = VGroup(
            target_legend_dot,
            target_legend_text,
        ).arrange(RIGHT, buff=0.15)

        # agents UAV legend
        agents_legend_dot = Dot(
            radius=0.08,
            color=WHITE,
        )

        agents_legend_text = MathTex(
            r"\text{chaser UAV }(p_i)", 
            color=WHITE, 
            font_size=36
        )

        agents_legend = VGroup(
            agents_legend_dot,
            agents_legend_text,
        ).arrange(RIGHT, buff=0.15)

        # p_star UAV legend
        p_star_legend_dot = Dot(
            radius=0.08,
            color=BLUE,
        )

        p_star_text = MathTex(
            r"\text{Desired point }(p^\star)", 
            color=BLUE, 
            font_size=36
        )

        p_star_legend = VGroup(
            p_star_legend_dot,
            p_star_text,
        ).arrange(RIGHT, buff=0.15)

        assumptions = Text(
            "System assumptions: "
            "! The System is Centralized\n" \
            "! Communication is Instant\n" \
            "! The system is not Optimal\n" \
            "! We only know target position",
            color=WHITE, font_size=24
        )

        # Stack p_star below target
        legend = VGroup(
            target_legend,
            agents_legend,
            p_star_legend,
            assumptions
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.15)

        legend.to_corner(UR, buff=0.3)

        self.add_fixed_in_frame_mobjects(legend)

        # Hide legend elements initially
        target_legend_dot.set_opacity(0)
        target_legend_text.set_opacity(0)
        agents_legend_dot.set_opacity(0)
        agents_legend_text.set_opacity(0)
        p_star_legend_dot.set_opacity(0)
        p_star_text.set_opacity(0)
        assumptions.set_opacity(0)

        self.next_slide()

        # -------------------------
        # Target UAV -> legend
        # -------------------------

        travelling_target_dot = Dot(
            radius=0.08,
            color=RED,
        )

        travelling_target_dot.move_to(
            self.camera.project_point(target_uav.get_center())
        )

        self.add_fixed_in_frame_mobjects(travelling_target_dot)

        self.play(
            travelling_target_dot.animate.move_to(
                target_legend_dot.get_center()
            ),
        )

        # Replace travelling dot with the actual legend dot
        self.play(
            target_legend_dot.animate.set_opacity(1),
            target_legend_text.animate.set_opacity(1),
            FadeOut(travelling_target_dot)
        )

        self.next_slide()

        # -------------------------
        # Create agents UAVs
        # -------------------------

        agents_list = []
        for agent_pos in agents_UAV_pos:
            agent_uav = (
                dot.copy()
                .set_color(WHITE)
                .move_to(agent_pos + target_uav.get_center())
            )
            agents_list.append(agent_uav)

        self.play(
            FadeIn(*agents_list)
        )

        # -------------------------
        # agents UAVs -> legend
        # -------------------------
        travelling_agents_dots = []
        for agent in agents_list:
            travelling_dot = Dot(
                radius=0.08,
                color=WHITE,
            )

            # Convert the 3D position to its current screen-space position
            travelling_dot.move_to(
                self.camera.project_point(agent.get_center())
            )

            self.add_fixed_in_frame_mobjects(travelling_dot)
            travelling_agents_dots.append(travelling_dot)

        # Animate all p_star copies toward the p_star legend
        self.play(
            *[
                travelling_dot.animate.move_to(
                    agents_legend_dot.get_center()
                )
                for travelling_dot in travelling_agents_dots
            ],
            run_time=1.5,
        )

        # Reveal the actual legend entry
        self.play(
            agents_legend_dot.animate.set_opacity(1),
            agents_legend_text.animate.set_opacity(1),
            *[
                FadeOut(travelling_dot)
                for travelling_dot in travelling_agents_dots
            ],
        )
       
        self.next_slide()

        # -------------------------
        # Create p_star UAVs
        # -------------------------

        p_stars_positions = fibonacci_points(3, 1)

        p_star_list = []
        for p_star_pos in p_stars_positions:
            p_star = (
                dot.copy()
                .set_color(BLUE)
                .move_to(p_star_pos + target_uav.get_center())
            )
            p_star_list.append(p_star)

        self.play(
            FadeIn(*p_star_list)
        )

        # -------------------------
        # p_star UAVs -> legend
        # -------------------------
        travelling_p_star_dots = []

        for p_star in p_star_list:
            travelling_dot = Dot(
                radius=0.08,
                color=BLUE,
            )

            # Convert the 3D position to its current screen-space position
            travelling_dot.move_to(
                self.camera.project_point(p_star.get_center())
            )

            self.add_fixed_in_frame_mobjects(travelling_dot)
            travelling_p_star_dots.append(travelling_dot)

        # Animate all p_star copies toward the p_star legend
        self.play(
            *[
                travelling_dot.animate.move_to(
                    p_star_legend_dot.get_center()
                )
                for travelling_dot in travelling_p_star_dots
            ],
            run_time=1.5,
        )

        # Reveal the actual legend entry
        self.play(
            p_star_legend_dot.animate.set_opacity(1),
            p_star_text.animate.set_opacity(1),
            *[
                FadeOut(travelling_dot)
                for travelling_dot in travelling_p_star_dots
            ],
        )

        self.next_slide()
        self.play(assumptions.animate.set_opacity(1))

    def objective(self):    
        # ============================================================
        # Helper: create the split-screen layout
        # ============================================================
        divider = Line(
            start=UP * 3.8,
            end=DOWN * 3.8,
            stroke_width=2,
        ).shift(RIGHT * 2.0)
        
        screen_width = config.frame_width
        screen_height = config.frame_height
        line_x = divider.get_x()

        active_title = Tex(
            r"\textbf{Equations}",
            font_size=30,
            color=YELLOW
        )

        active_title_middle_x = (line_x + screen_width / 2) / 2
        active_title.set_x(active_title_middle_x)  
        active_title.shift(UP * 3.1)
        active_title.set_opacity(0)
        divider.set_opacity(0)
        self.add_fixed_in_frame_mobjects(divider, active_title)
        self.play(
            Create(divider),
            FadeIn(active_title)
        )

        # -------------------------
        # Scene setup
        # -------------------------
        axes = ThreeDAxes()

        x_label = axes.get_x_axis_label(Tex("x"))
        y_label = axes.get_y_axis_label(Tex("y")).shift(UP * 1.8)
        z_label = axes.get_z_axis_label(Tex("z")).rotate_about_origin(PI)

        axes_group = VGroup(
            axes,
            x_label,
            y_label,
            z_label,
        )

        self.set_camera_orientation(
            phi=CAMERA_PHI,
            theta=CAMERA_THETA,
            zoom=1,
        )

        self.add(axes_group)

        # Target UAV legend
        target_legend_dot = Dot(
            radius=0.08,
            color=RED,
        )

        target_legend_text = MathTex(
            r"\text{target UAV }(p_T)",
            color=RED,
            font_size=36,
        )
        
        target_legend = VGroup(
            target_legend_dot,
            target_legend_text,
        ).arrange(RIGHT, buff=0.15)

        # agents UAV legend
        agents_legend_dot = Dot(
            radius=0.08,
            color=WHITE,
        )

        agents_legend_text = MathTex(
            r"\text{chaser UAV }(p_i)", 
            color=WHITE, 
            font_size=36
        )

        agents_legend = VGroup(
            agents_legend_dot,
            agents_legend_text,
        ).arrange(RIGHT, buff=0.15)

        # p_star UAV legend
        p_star_legend_dot = Dot(
            radius=0.08,
            color=BLUE,
        )

        p_star_text = MathTex(
            r"\text{Desired point }(p^\star)", 
            color=BLUE, 
            font_size=36
        )

        p_star_legend = VGroup(
            p_star_legend_dot,
            p_star_text,
        ).arrange(RIGHT, buff=0.15)

            # Stack p_star below target
        legend = VGroup(
            target_legend,
            agents_legend,
            p_star_legend,
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.15)

        legend.to_corner(UR, buff=0.3)

        self.add_fixed_in_frame_mobjects(legend) 
        # -------------------------
        # UAVs
        # -------------------------
        origin = OUT

        target_uav = (
            Dot3D()
            .set_color(RED)
            .move_to(origin)
        )

        agents = VGroup(*[
            Dot3D()
            .set_color(WHITE)
            .move_to(origin + np.array(pos))
            for pos in agents_UAV_pos
        ])

        p_stars_positions = fibonacci_points(3, 1)

        p_stars = VGroup(*[
            Dot3D()
            .set_color(BLUE)
            .move_to(origin + np.array(pos))
            for pos in p_stars_positions
        ])

        self.add(
            target_uav,
            agents,
            p_stars,
        )

        # -------------------------
        # Transition
        # -------------------------
        
        # Camera-left displacement
        left = np.array([
            np.sin(CAMERA_THETA),
            -np.cos(CAMERA_THETA),
            0,
        ])

        AXIS_ORIGIN = 2.5 * left

        scene_group = VGroup(
            axes_group,
            target_uav,
            agents, 
            p_stars,
        )

        self.play(
            legend.animate.next_to(divider, LEFT, buff=0.3).set_y(
                screen_height / 2 - 0.3 - legend.height / 2
            ),
            scene_group.animate.shift(AXIS_ORIGIN),
            active_title.animate.set_opacity(1),
            divider.animate.set_opacity(1),
            run_time=1.5,
        )   

        animation = []
        for p_star, agent in zip(p_stars, agents):
            animation.append(agent.animate.move_to(p_star))

        # Objective equation
        objective_eq = MathTex(            
            r"\lim_{t \to \infty} p_i(t) = p_i^\star, \quad \forall i",
            color=WHITE,
            font_size=36,
        ).next_to(active_title, DOWN, buff=0.18)
        self.add_fixed_in_frame_mobjects(objective_eq)
        self.play(
            *animation,
            FadeIn(objective_eq)
        )

        # -------------------------
        # Rotate UAVs around Z
        # -------------------------

        formation = VGroup(
            target_uav,
            p_stars,
            agents,
        )

        center = target_uav.get_center().copy()
        self.play(
            formation.animate.shift(RIGHT * 2),
            run_time=1.5,
        )
 
        def rotate_formation(mob, alpha):
            angle = alpha * 2 * PI

            # Rotate the target position around the origin
            x, y, z = center

            target_x = (
                x * np.cos(angle)
                - y * np.sin(angle)
            )

            target_y = (
                x * np.sin(angle)
                + y * np.cos(angle)
            )

            new_target_pos = np.array([
                target_x,
                target_y,
                z,
            ])+ AXIS_ORIGIN

            target_uav.move_to(new_target_pos)

            # Keep every agent at the same relative
            # position with respect to the target
            for p_star, relative_pos, agent in zip(
                p_stars,
                p_stars_positions,
                agents
            ):
                relative_pos = relative_pos.flatten()

                p_star.move_to(
                    new_target_pos + relative_pos
                )

                agent.move_to(
                    new_target_pos + relative_pos
                )

        self.next_slide(loop=True)
        self.play(
            UpdateFromAlphaFunc(
                formation,
                rotate_formation,
            ),
            run_time=4,
            rate_func=linear,
        )

        self.next_slide()

        target_on_axis = np.array([0, 0, 1.5] + AXIS_ORIGIN)
        animations = []
        i = 0
        for p_star, relative_pos, agent in zip(p_stars, p_stars_positions, agents):
            relative_pos = relative_pos.flatten()
            animations.append(p_star.animate.move_to(target_on_axis + relative_pos))
            animations.append(agent.animate.move_to(agents_UAV_pos[i] + AXIS_ORIGIN))
            i += 1

        self.play(
            target_uav.animate.move_to(target_on_axis),
            *animations
        )

        remaining_agent = agents[2]
        remaining_p_star = p_stars[2]
        focus_group = VGroup(
            remaining_p_star,
            remaining_agent,
            target_uav,
        )

        self.play(
            FadeOut(*p_stars[:2]),
            FadeOut(*agents[:2]),
            run_time=1,
        )

        focus_center = focus_group.get_center()

        self.move_camera(
            zoom=2,
            frame_center=focus_center - left*1.5,
            run_time=2,
        )

        # ----------------
        # d vector
        # ----------------

        d = always_redraw(
            lambda: Arrow3D(
                target_uav.get_center(),
                remaining_agent.get_center(),
                thickness=0.035,
                color=YELLOW,
            )
        )

        d_star = Arrow3D(target_uav.get_center(), remaining_p_star.get_center(), thickness=0.035, color=YELLOW)

        # p_T_label = MathTex(
        #     r"p_T",
        #     color=RED,
        #     font_size=32,
        # )
        # label_screen_point = self.camera.project_point(target_uav.get_center())
        # p_T_label.move_to(label_screen_point + UP * 0.35)
        # self.add_fixed_in_frame_mobjects(p_T_label)

        # agent_label = MathTex(
        #     r"p",
        #     color=WHITE,
        #     font_size=32,
        # )
        # label_screen_point = self.camera.project_point(remaining_agent.get_center())
        # agent_label.move_to(label_screen_point + UP * 0.35)
        # self.add_fixed_in_frame_mobjects(agent_label)

        # p_star_label = MathTex(
        #     r"p^\star",
        #     color=BLUE,
        #     font_size=32,
        # )
        # label_screen_point = self.camera.project_point(remaining_p_star.get_center())
        # p_star_label.move_to(label_screen_point + UP * 0.35)
        # self.add_fixed_in_frame_mobjects(p_star_label)

        self.play(
            FadeIn(d),
        #     # FadeIn(d_star),
        #     FadeIn(p_T_label),
        #     FadeIn(agent_label),
        #     FadeIn(p_star_label),
        )

        # ----------------
        # Equations Legend
        # ----------------

        d_eq = MathTex(
            r"p-p_T = d = rq",
            color=WHITE,
            font_size=36,
        )

        r_value = DecimalNumber(
            0,
            num_decimal_places=2,
            color=WHITE,
            font_size=36,
        )

        def update_r(m):
            r_vec = (
                remaining_agent.get_center()
                - target_uav.get_center()
            )
            m.set_value(np.linalg.norm(r_vec))
            self.add_fixed_in_frame_mobjects(m)

        r_value.add_updater(update_r)

        r_eq = VGroup(
            MathTex(r"\|p-p_T\| = r = "),
            r_value,
        )
        r_eq.arrange(RIGHT, buff=0.1)


        qx_value = DecimalNumber(
            0,
            num_decimal_places=2,
            include_sign=True,
            color=WHITE,
            font_size=36,
        )

        qy_value = DecimalNumber(
            0,
            num_decimal_places=2,
            include_sign=True,
            color=WHITE,
            font_size=36,
        )

        qz_value = DecimalNumber(
            0,
            num_decimal_places=2,
            include_sign=True,
            color=WHITE,
            font_size=36,
        )


        def get_q():
            d_vec = (
                remaining_agent.get_center()
                - target_uav.get_center()
            )

            r = np.linalg.norm(d_vec)

            return d_vec / r


        def update_qx(m):
            m.set_value(get_q()[0])

            self.add_fixed_in_frame_mobjects(m)

        def update_qy(m):
            m.set_value(get_q()[1])
            self.add_fixed_in_frame_mobjects(m)

        def update_qz(m):
            m.set_value(get_q()[2])
            self.add_fixed_in_frame_mobjects(m)


        qx_value.add_updater(update_qx)
        qy_value.add_updater(update_qy)
        qz_value.add_updater(update_qz)

        q_eq = VGroup(
            MathTex(r"q = ["),
            qx_value,
            MathTex(","),
            qy_value,
            MathTex(","),
            qz_value,
            MathTex("]"),
        )

        q_eq.arrange(RIGHT, buff=0.1).scale(0.8)

        # Re-arrange whenever DecimalNumbers change size
        def update_q_eq(m):
            m.arrange(RIGHT, buff=0.1)
            m.next_to(r_eq, DOWN)

        q_eq.add_updater(update_q_eq)
        equations = VGroup(
            d_eq,
            r_eq,
        )

        equations.arrange(
            DOWN,
            aligned_edge=LEFT,
            buff=0.18,
        )

        equations.next_to(
            objective_eq,
            DOWN,
            buff=0.18,
        ).align_to(objective_eq, LEFT)

        self.add_fixed_in_frame_mobjects(equations, q_eq)
        self.play(FadeIn(equations), FadeIn(q_eq))

        self.next_slide()

        # --------------------------------------------------
        # Move radially inward
        # --------------------------------------------------

        target_pos = target_uav.get_center()
        agent_pos = remaining_agent.get_center()

        d_vec = agent_pos - target_pos
        r = np.linalg.norm(d_vec)
        q = d_vec / r

        r -= 1

        self.play(
            remaining_agent.animate.move_to(
                target_pos + r * q
            ),
            run_time=3,
        )


        # --------------------------------------------------
        # Rotate q by pi/4 while keeping r constant
        # --------------------------------------------------

        theta = ValueTracker(0)

        def rotate_q(angle):
            R = np.array([
                [np.cos(angle), -np.sin(angle), 0],
                [np.sin(angle),  np.cos(angle), 0],
                [0,              0,             1],
            ])
            return R @ q

        remaining_agent.add_updater(
            lambda m: m.move_to(
                target_pos + r * rotate_q(theta.get_value())
            )
        )

        self.play(
            theta.animate.set_value(np.pi),
            run_time=3,
            rate_func=linear,
        )

        remaining_agent.clear_updaters()

        # --------------------------------------------------
        # desired values
        # --------------------------------------------------

        r_star = (
            MathTex(r"r^\star = 1", color=WHITE, font_size=36)
        )

        q_star = (
            MathTex(r"q^\star = [1.2, -0.8, 2]", color=WHITE, font_size=36)
        )
        equations_2 = VGroup(
            r_star,
            q_star 
        )
        equations_2.arrange(
            DOWN,
            aligned_edge=LEFT,
            buff=0.18,
        )
        equations_2.next_to(
            q_eq,  
            DOWN, 
            buff=0.18
        ).align_to(q_eq, LEFT)

        self.add_fixed_in_frame_mobjects(equations_2)
        self.play(
            FadeIn(equations_2)
        )

        self.wait(2)

        new_objective_eq_1 = MathTex(
            r"\lim_{t\to\infty} r_i(t) = r^\star, \quad \forall i",
            color=WHITE,
            font_size=36,
        )

        new_objective_eq_2 = MathTex(
            r"\lim_{t\to\infty} q_i(t) = q_i^\star, \quad \forall i",
            color=WHITE,
            font_size=36,
        )

        p_star_2_label = (
            MathTex(
                r"p^\star = p_T + r^\star q_i^\star",
                color=WHITE,
                font_size=36,
            )
        )

        equations_3 = VGroup(
            new_objective_eq_1,
            new_objective_eq_2,
            p_star_2_label,
        )
        equations_3.arrange(
            DOWN,
            aligned_edge=LEFT,
            buff=0.18,
        )
        equations_3.next_to(
            q_star, 
            DOWN, 
            buff=0.18
        ).align_to(q_star, LEFT)
        self.add_fixed_in_frame_mobjects(equations_3)

        self.play(
            FadeIn(equations_3),
        )

        self.next_slide()
        self.play(
            remaining_agent.animate.move_to(p_star.get_center())
        )
        self.wait(5)
        
    class FibonacciSphere(ThreeDSlide):
        RADIUS = 3.0
        MAX_POINTS = 2000

        def construct(self):
            # ============================================================
            # Helper: create the split-screen layout
            # ============================================================
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
            self.add(
                divider,
                active_title
            )

            # -------------------------
            # Scene setup
            # -------------------------
            axes = ThreeDAxes()

            x_label = axes.get_x_axis_label(Tex("x"))
            y_label = axes.get_y_axis_label(Tex("y")).shift(UP * 1.8)
            z_label = axes.get_z_axis_label(Tex("z")).rotate_about_origin(PI)

            axes_group = VGroup(
                axes,
                x_label,
                y_label,
                z_label,
            )

            self.set_camera_orientation(
                phi=CAMERA_PHI,
                theta=CAMERA_THETA,
                zoom=1,
            )

            self.add(axes_group)

            # Camera-left displacement
            left = np.array([
                np.sin(CAMERA_THETA),
                -np.cos(CAMERA_THETA),
                0,
            ])

            AXIS_ORIGIN = 2.5 * left

            p_stars_positions = fibonacci_points(3, 1)

            target_uav = (
                Dot3D()
                .set_color(RED)
                .move_to(OUT)
            )

            p_stars = VGroup(*[
                Dot3D()
                .set_color(BLUE)
                .move_to(target_uav.get_center() + np.array(pos))
                for pos in p_stars_positions
            ])

            scene_group = VGroup(
                axes_group,
                target_uav,
                p_stars,
            ).shift(AXIS_ORIGIN)

            self.add(
                scene_group,
                target_uav,
                p_stars
            )
        
            # ----------------------------------------------------------
            # Fixed UI
            # ----------------------------------------------------------

            n_label = Text(
                "P =",
                font_size=32,
            )

            n_value = DecimalNumber(
                3,
                num_decimal_places=0,
                font_size=32,
            )

            counter = VGroup(
                n_label,
                n_value,
            ).arrange(RIGHT, buff=0.12)
        
            n_tracker = ValueTracker(3)

            def update_counter(m):
                m.set_value(int(round(n_tracker.get_value())))
                self.add_fixed_in_frame_mobjects(m)

            n_value.add_updater(update_counter)

            golden_phi = MathTex(
                r"\varphi = \frac{1+\sqrt{5}}{2}",
                color=WHITE,
                font_size=36,
            )

            legend = VGroup(
                counter,
                golden_phi,
            ).arrange(RIGHT, buff=1.0)

            eq_fib = MathTex(
                r"""
                \begin{aligned}
                &(k \in \{0,\dots,P-1\}) \\
                &\mathrm{lat}_k =
                    \arcsin\left(
                        \frac{2k}{P-1}-1
                    \right) \\
                &\mathrm{lon}_k =
                    2\pi k\varphi^{-1}
                \end{aligned}
                """,
                color=WHITE,
                font_size=36,
            )

            legend = VGroup(
                legend,
                eq_fib,
            ).arrange(
                DOWN,
                buff=0.2,
                aligned_edge=LEFT,
            ).next_to(active_title, DOWN, buff=0.18)
            self.add_fixed_in_frame_mobjects(legend)

            R = 1.0
            sphere = Sphere(
                radius=R,
                resolution=(32, 64),
                fill_opacity=0.10,
                stroke_opacity=0.35,
            ).move_to(AXIS_ORIGIN)

            # Equatorial reference circle
            equator = Circle(
                radius=R,
                stroke_opacity=0.22,
            ).rotate(PI / 2, axis=RIGHT).move_to(AXIS_ORIGIN)

            animation = []
            for p_star in p_stars:
                animation.append(p_star.animate.shift(IN))

            self.play(
                *animation,
                FadeOut(target_uav),
                FadeIn(legend),
            )

            self.play(
                FadeIn(sphere),
                Create(equator),
            )

            self.next_slide()

            self.play(
                FadeOut(p_stars),
                sphere.animate.set_opacity(0.05),
                FadeOut(equator)
            )

            self.play(
                sphere.animate.scale(self.RADIUS / sphere.radius)
            )

            # ----------------------------------------------------------
            # 2000 reusable Dot objects
            # ----------------------------------------------------------

            dots = VGroup(*[
                Dot(
                    radius=0.05,
                    color=BLUE,
                )
                for _ in range(self.MAX_POINTS)
            ])

            self.add(dots)

            # Initially only 3 are visible.
            for dot in dots[3:]:
                dot.set_opacity(0)

            # ----------------------------------------------------------
            # The important part:
            #
            # Every frame, recompute the ENTIRE Fibonacci lattice for
            # the current N.
            # ----------------------------------------------------------

            last_value = int(round(n_tracker.get_value()))
            def update_lattice(group):
                nonlocal last_value

                n = int(round(n_tracker.get_value()))

                points = fibonacci_points(n, self.RADIUS)

                # Move the complete current lattice.
                for i in range(n):
                    group[i].move_to(points[i] + AXIS_ORIGIN)

                # Reveal newly created points.
                if n > last_value:
                    group[last_value:n].set_opacity(1)

                last_value = n

            dots.add_updater(update_lattice)

            # ----------------------------------------------------------
            # 3 -> 500
            # ----------------------------------------------------------

            self.play(
                n_tracker.animate.set_value(500),
                run_time=10,
                rate_func=linear,
            )

            self.wait(1)

            # ----------------------------------------------------------
            # 500 -> 1000
            # ----------------------------------------------------------

            self.play(
                n_tracker.animate.set_value(1000),
                run_time=1.0,
                rate_func=linear,
            )

            self.wait(1)

            # ----------------------------------------------------------
            # 1000 -> 2000
            # ----------------------------------------------------------

            self.play(
                n_tracker.animate.set_value(2000),
                run_time=1.0,
                rate_func=linear,
            )

            self.wait(2)
            dots.clear_updaters()

            # ----------------------------------------------------------
            # Add execution time plot
            # ----------------------------------------------------------
            fib_time = SVGMobject("assets/svg/fibonacci_exec_time.svg")
            fib_time.set(width=4.5)
            self.add_fixed_in_frame_mobjects(fib_time)

            legend_bottom = legend.get_bottom()[1]
            screen_bottom = -config.frame_height / 2

            padding = 0.3

            fib_time.set_x(active_title_middle_x)
            fib_time.set_y(
                (legend_bottom + screen_bottom + padding) / 2
            )

            self.add(fib_time)
            self.next_slide()

    class ControllerDesign(Slide):
        def construct(self):
            # ============================================================
            # Helper: create the split-screen layout
            # ============================================================

            divider = Line(
                start=UP * 3.8,
                end=DOWN * 3.8,
                stroke_width=2,
            ).shift(RIGHT * 2.0)

            screen_width = config.frame_width
            line_x = divider.get_x()

            stored_title_middle_x = (line_x + screen_width / 2) / 2
    
            stored_title = Tex(
                r"\textbf{Stored equations}",
                font_size=30,
                color=YELLOW
            )
            stored_title.set_x(stored_title_middle_x)  
            stored_title.shift(UP * 3.1)

            active_title = Tex(
                r"\textbf{Current derivation}",
                font_size=30,
                color=YELLOW
            )

            active_title_middle_x = (config.frame_width / -2 + divider.get_x()) / 2
            active_title.set_x(active_title_middle_x)  
            active_title.shift(UP * 3.1)

            def show_split_screen():
                self.play(
                    Create(divider),
                    FadeIn(active_title),
                    FadeIn(stored_title),
                )

            stored_y = stored_title.get_bottom()[1] - 0.35
            stored_x = stored_title.get_bottom()[0]
            def store_equation(eq, buff=0.5):
                nonlocal stored_y

                eq_copy = eq.copy()
                eq_copy.scale(0.65)

                # Fixed horizontal position
                eq_copy.set_x(stored_x)

                # Put it at the current vertical position
                eq_copy.set_y(stored_y)

                self.play(FadeIn(eq_copy))

                # Move downward for the next equation
                stored_y -= eq_copy.height + buff

                return eq_copy

            # ------------------------------------------------------------
            # Slide 1 — Controller architecture
            # ------------------------------------------------------------

            # ------------------ Architecture blocks ------------------
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
            self.play(
                LaggedStart(
                    *[FadeIn(box) for box in architecture_boxes],
                    lag_ratio=0.1,
                )
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

            self.play(
                LaggedStart(
                    *[GrowArrow(arrow) for arrow in architecture_arrows],
                    lag_ratio=0.1,
                )
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

            self.play(FadeIn(architecture_labels))

            architecture = VGroup(
                architecture_boxes,
                architecture_arrows,
                architecture_labels
            )

            # ------------------------------------------------------------
            # Slide 2 — Sphere projection
            # ------------------------------------------------------------
            self.next_slide()

            self.play(
                architecture.animate.scale(0.35)
            )

            self.play(
                architecture.animate.to_corner(DL, buff=0.3)
            )

            show_split_screen()

            indicate_scale = 1.4
            self.play(
                a_input_p.animate.scale(indicate_scale),
                a_input_p_star.animate.scale(indicate_scale),
                label_p_star.animate.scale(indicate_scale),
                label_p_input.animate.scale(indicate_scale)
            )

            proj_eq_dimensions = MathTex(
                r"\Pi_{\Gamma^2}^{R,P} : \mathbb{R}^3 \to \Gamma^2(R,P)",
                font_size=34
            )
            proj_eq = MathTex(
                r"""
                    \Pi_{\Gamma^2}^{R,P}(p)
                    \;=\;
                    P + R \frac{p - P}{\|p - P\|},
                    \qquad p \neq P
                """,
                font_size=34
            )

            proj_eq_label = VGroup(
                proj_eq_dimensions,
                proj_eq
            ).arrange(DOWN, buff=0.18).next_to(active_title, DOWN, buff=0.18)

            self.play(FadeIn(proj_eq_label))

            self.next_slide()

            proj_eq_examples = MathTex(
                r"Ex: q_i = \Pi_{\Gamma^2}^{1,0}(d_i), \quad q_i^\star =\Pi_{\Gamma^2}^{1,0}(p_i^\star)",
                font_size=34
            ).next_to(proj_eq_label, DOWN, buff=0.4)

            self.play(FadeIn(proj_eq_examples))
            store_equation(proj_eq)

            # ------------------------------------------------------------
            # Slide 3 — Geometric controller
            # ------------------------------------------------------------
            self.next_slide()

            self.play(
                a_input_p.animate.scale(1.0/indicate_scale),
                a_input_p_star.animate.scale(1.0/indicate_scale),
                label_p_star.animate.scale(1.0/indicate_scale),
                label_p_input.animate.scale(1.0/indicate_scale),
                FadeOut(proj_eq_label),
                FadeOut(proj_eq_examples),
                geometric.animate.scale(indicate_scale),
            )

            dynamics = MathTex(
                r"\dot{q}_i = -k_{geo} S(e_{q_i}) q_i \text{ (11)}",
                font_size=38
            )

            geometric_err = MathTex(
                r"e_{q_i} = \frac{S(q_i^\star) q_i}{\sqrt{2(1 + q_i^\top q_i^\star)}} \text{ (13)}",
                font_size=34
            )

            dynamics.set_x(-screen_width / 4)
            dynamics.set_y(2.5)

            geometric_err.set_x(screen_width / 4)
            geometric_err.set_y(2.5)

            proposition_1 = Tex(
                r"\textbf{Proposition:} Given the error function (13), "
                r"the equilibrium $q_i = q_i^\star$ of the closed-loop system (11) "
                r"is almost globally asymptotically stable on $\mathbb{S}^2$.",
                font_size=36,
                color=WHITE,
                tex_to_color_map={
                    r"\textbf{Proposition:}": YELLOW
                }
            ).move_to(UP)

            content = VGroup(
                dynamics,
                geometric_err,
                proposition_1
            )

            background = RoundedRectangle(
                corner_radius=0.2,
                width=screen_width * 0.9,
                height=2.5,
                stroke_color=WHITE,
                stroke_width=2,
                fill_color=BLACK,
                fill_opacity=1.0,
            )

            background.surround(content, buff=0.3)

            # Rectangle above anything previously rendered
            background.set_z_index(10)

            # Content above the rectangle
            content.set_z_index(11)

            self.play(
                FadeIn(background),
                FadeIn(dynamics),
                FadeIn(geometric_err),
                FadeIn(proposition_1),
            )

            self.next_slide()

            self.play(
                FadeOut(background),
                FadeOut(dynamics),
                FadeOut(geometric_err),
                FadeOut(proposition_1),
            )

            stored_dynamics = store_equation(dynamics)
            store_equation(geometric_err)


            # ------------------------------------------------------------
            # Slide 4 — Lyapunov stability
            # ------------------------------------------------------------
            self.next_slide()
        
            # Configuration error
            psi = MathTex(
                r"\Psi_i(q_i, q_i^\star) = 2 - 2 \sqrt{\frac{1 + q_i^\top q_i^\star}{2}} \text{(12)}",
                font_size=34
            ).next_to(active_title, DOWN, buff=0.35)

            # ------------- Step 1: Lyapunov candidate -------------
            v_eq = MathTex(
                r"V(q_i)=\Psi_i(q_i,q_i^\star)",
                font_size=40
            ).next_to(psi, DOWN, buff=0.35)

            v_nonneg = MathTex(
                r"V(q_i)\geq 0, \qquad V(q_i)=0 \iff q_i=q_i^\star",
                font_size=34
            ).next_to(v_eq, DOWN, buff=0.18)

            dot_psi = MathTex(
                r"\dot \Psi = e_{q_i}^\top \omega_i",
                font_size=34
            ).next_to(v_nonneg, DOWN, buff=0.18)
        
            self.play(
                FadeIn(psi),
                FadeIn(v_eq),
                FadeIn(v_nonneg),
                FadeIn(dot_psi),
            )

            # ------------- Step 2: Start from the sphere kinematics -------------
            self.next_slide()

            stored_dot_psi = store_equation(dot_psi)

            self.play(
                FadeOut(psi),
                FadeOut(v_eq),
                FadeOut(v_nonneg),
                FadeOut(dot_psi),
            )

            lhs = MathTex("\dot q_i", font_size=40) 
            sphere_equal = MathTex("=", font_size=40) 
            rhs = MathTex("S(\omega_i)","q_i", font_size=40)

            sphere_dynamic = VGroup(lhs, sphere_equal, rhs)
            sphere_dynamic.arrange(RIGHT, buff=0.15).next_to(active_title, DOWN, buff=0.35)

            self.play(FadeIn(sphere_dynamic))
        
            # ------------- Step 3: Identify the angular velocity -------------
            self.next_slide()
        
            new_lhs = MathTex(
                "-k_{geo}S(e_{q_i})", "q_i",
                font_size=40
            ).next_to(sphere_equal, LEFT) 

            self.play(
                TransformMatchingTex(
                    lhs,
                    new_lhs,
                    run_time = 3
                ),
                Indicate(stored_dynamics)
            )
            lhs = new_lhs

            new_lhs = MathTex(
                "-k_{geo}S(e_{q_i})",
                font_size=40
            ).next_to(sphere_equal, LEFT) 

            new_rhs = MathTex(
                "S(\omega_i)",
                font_size=40
            ).next_to(sphere_equal, RIGHT)

            self.play(
                TransformMatchingTex(
                    lhs,
                    new_lhs,
                    run_time = 3
                ),
                TransformMatchingTex(
                    rhs,
                    new_rhs,
                    run_time = 3
                ),
            )
            lhs_1 = new_lhs
            rhs_1 = new_rhs

            lhs = MathTex(
                "-k_{geo}", "S(", "e_{q_i}", ")",
                font_size=40
            ).next_to(sphere_equal, LEFT) 

            rhs = MathTex(
                "S(", "\omega_i", ")",
                font_size=40
            ).next_to(sphere_equal, RIGHT)

            new_lhs = MathTex(
                "-k_{geo}", "e_{q_i}",
                font_size=40
            ).next_to(sphere_equal, LEFT) 

            new_rhs = MathTex(
                "\omega_i",
                font_size=40
            ).next_to(sphere_equal, RIGHT)

            self.play(
                FadeOut(lhs_1),
                FadeOut(rhs_1),
                TransformMatchingTex(
                    lhs,
                    new_lhs,
                    run_time = 3
                ),
                TransformMatchingTex(
                    rhs,
                    new_rhs,
                    run_time = 3
                ),
            )
            lhs = new_lhs
            rhs = new_rhs

            sphere_dynamic = VGroup(lhs, sphere_equal, rhs)
            sphere_dynamic.arrange(RIGHT, buff=0.15).next_to(active_title, DOWN, buff=0.35)

            stored_sphere_dynamic = store_equation(sphere_dynamic)

            # ------------- Step 4: Differentiate the Lyapunov function -------------
            self.next_slide()
            self.play(FadeOut(sphere_dynamic))
        
            lhs = MathTex("\dot \Psi_i", font_size=40) 
            psi_equal = MathTex("=", font_size=40) 
            rhs = MathTex("e_q^{\\top}", "\omega", font_size=40)

            psi_dot = VGroup(lhs, psi_equal, rhs)
            psi_dot.arrange(RIGHT, buff=0.15).next_to(active_title, DOWN, buff=0.35)
            self.play( 
                Indicate(stored_dot_psi),
                FadeIn(psi_dot)
            )

            new_rhs_1 = MathTex(
                "(-k_{geo} e_{q_i})",
                font_size=40
            ).next_to(rhs[0], RIGHT) 

            self.play(
                Transform(rhs[1], new_rhs_1, run_time=3),
                Indicate(stored_sphere_dynamic)
            )

            new_rhs = MathTex(
                "-k_{geo}\|e_{q_i}\|^2",
                font_size=40
            ).next_to(psi_equal, RIGHT) 

            self.play(
                Transform(rhs, new_rhs, run_time=3),
            )
            rhs = new_rhs

            ineq_zero = MathTex(
                "\\leq 0",
                font_size=40
            ).next_to(rhs, RIGHT) 

            v_dot = MathTex(
                "\\dot V =",
                font_size=40
            ).next_to(lhs, LEFT) 

            self.play(
                FadeIn(v_dot),
                FadeIn(ineq_zero),
            )

            v_proof = MathTex(
                "\\dot V = 0 \\Leftrightarrow e_q = 0",
                font_size=40
            ).next_to(psi_equal, DOWN) 
                
            v_proof_1 = MathTex(
                """
                    q_i = q_i^\\star
                    \\qquad \\text{or} \\qquad
                    q_i = - q_i^\\star.
                """,
                font_size=40
            ).next_to(v_proof, DOWN) 

            self.play(
                FadeIn(v_proof),
                FadeIn(v_proof_1),
            )

            self.next_slide()

    class APF(ThreeDSlide):
        def construct(self):
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
            target_uav = dot.copy().set_color(RED).shift(RIGHT-LEFT + OUT + AXIS_ORIGIN)

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
                apf.animate.scale(indicate_scale),
                FadeIn(target_uav),
                FadeIn(*agents_list)
            )   

            proj_eq = MathTex(
                r"\Pi_{\Gamma^2}^{1,0}(p_i) = \frac{p}{\|p\|}",
                font_size=28
            ).next_to(active_title, DOWN, buff=0.18)
            self.add_fixed_in_frame_mobjects(proj_eq)

            animation = []

            for agent in agents_list:
                current_pos = agent.get_center() - AXIS_ORIGIN

                # Project onto unit sphere centered at origin
                target_pos = current_pos / np.linalg.norm(current_pos)

                # Move from current position to projected position
                animation.append(
                    agent.animate.move_to(target_pos + AXIS_ORIGIN)
                )

            self.play(
                FadeOut(target_uav),
                FadeIn(proj_eq),
                *animation
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
            ).move_to(AXIS_ORIGIN)

            # Equatorial reference circle
            equator = Circle(
                radius=R,
                stroke_opacity=0.22,
            ).rotate(PI / 2, axis=RIGHT).move_to(AXIS_ORIGIN)

            self.play(
                FadeIn(sphere),
                Create(equator),
            )

            # # ============================================================
            # # UAV POINTS
            # # ============================================================
            R = 2.5
            uav_i = agents_list[0]
            uav_j = agents_list[1]

            # Get vectors relative to sphere center
            v_i = uav_i.get_center() - AXIS_ORIGIN
            v_j = uav_j.get_center() - AXIS_ORIGIN

            # Normalize and place exactly on sphere surface
            q_i = AXIS_ORIGIN + R * v_i / np.linalg.norm(v_i)
            q_j = AXIS_ORIGIN + R * v_j / np.linalg.norm(v_j)
        
            self.play(
                FadeOut(agents_list[2]),
                sphere.animate.scale(R),
                equator.animate.scale(R),
                uav_i.animate.move_to(q_i),
                uav_j.animate.move_to(q_j),
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

            self.play(
                uav_i.animate.set_color(RED),
                uav_j.animate.set_color(BLUE),
                FadeIn(label_i), 
                FadeIn(label_j)
            )

            # ============================================================
            # GEODESIC DISTANCE
            # ============================================================

            # Great-circle interpolation between q_i and qj
            a = q_i
            b = q_j

            angle = np.arccos(
                np.clip(
                    np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)),
                    -1,
                    1,
                )
            )

            def geodesic_point(t):
                if abs(angle) < 1e-6:
                    p = a
                else:
                    p = (
                        np.sin((1 - t) * angle) / np.sin(angle) * a
                        + np.sin(t * angle) / np.sin(angle) * b
                    )
                return ORIGIN + p

            geodesic = VMobject()

            geodesic.set_points_smoothly([
                geodesic_point(t)
                for t in np.linspace(0, 1, 50)
            ])

            # Put label at the middle of the geodesic
            label_point = geodesic_point(0.5)

            geodesic_label = MathTex(
                r"\text{dist}_{q_i,q_j}",
                font_size=29
            )

            # This is a fixed-in-frame label, so convert the 3D point
            # to screen coordinates first.
            label_screen_point = self.camera.project_point(label_point)

            geodesic_label.move_to(label_screen_point + UP * 0.35)

            self.add_fixed_in_frame_mobjects(geodesic_label)

            self.play(
                Create(geodesic),
                FadeIn(geodesic_label),
            )

            # ============================================================
            # NORMAL n_i
            # ============================================================

            normal_eq = MathTex(
                r"n_i=\frac{q_i-p_T}{\|q_i-p_T\|}",
                font_size=27
            ).next_to(proj_eq, DOWN, buff=0.18)

            n_i = q_i / np.linalg.norm(q_i)
            normal_arrow = Arrow3D(
                start=q_i,
                end=q_i + n_i,
                thickness=0.025,
                height=0.12,
                base_radius=0.045,
            )

            normal_label = MathTex(
                r"n_i",
                font_size=27
            )

            normal_label_point = self.camera.project_point(q_i+n_i)

            normal_label.move_to(normal_label_point + UP * 0.35)

            self.add_fixed_in_frame_mobjects(
                normal_label,
                normal_eq,
            )

            self.play(
                GrowFromPoint(normal_arrow, q_i),
                FadeIn(normal_label),
                FadeIn(normal_eq),
            )

            # ============================================================
            # RELATIVE DISPLACEMENT q_i - q_j
            # ============================================================

            relative = q_i - q_j

            relative_eq = MathTex(
                r"\Delta_{ij}=q_i - q_j",
                font_size=27
            ).next_to(normal_eq, DOWN, buff=0.18)

            relative_arrow = Arrow3D(
                start=q_i,
                end=q_i + relative,
                thickness=0.035,
                height=0.14,
                base_radius=0.055,
            )

            relative_label = MathTex(
                r"q_i-q_j",
                font_size=25
            )

            relative_label_point = self.camera.project_point(q_i + relative)

            relative_label.move_to(relative_label_point + UP * 0.35)

            self.add_fixed_in_frame_mobjects(
                relative_label,
                relative_eq
            )

            self.play(
                FadeIn(relative_eq),
                FadeIn(relative_arrow),
                FadeIn(relative_label),
            )

            # ============================================================
            # TANGENTIAL PROJECTION
            # ============================================================

            tangent_eq = MathTex(
                r"\Delta_{ij}^{tan}"
                r"=\Delta_{ij}"
                r"-\big(\Delta_{ij}^\top n_i\big)n_i",
                font_size=24
            ).next_to(relative_eq, DOWN, buff=0.18)

            # Tangential component
            tangent = relative - np.dot(relative, n_i) * n_i

            # Scale only for visualization
            tangent_visual = tangent / np.linalg.norm(tangent)

            tangent_arrow = Arrow3D(
                start=q_i,
                end=q_i + tangent_visual,
                thickness=0.035,
                height=0.17,
                base_radius=0.065,
            )

            tangent_label = MathTex(
                r"\Delta_{ij}^{tan}",
                font_size=27
            )

            tangent_label_point = self.camera.project_point(q_i + tangent_visual)

            tangent_label.move_to(tangent_label_point + UP * 0.35)

            self.add_fixed_in_frame_mobjects(
                tangent_label,
                tangent_eq,
            )

            self.play(
                FadeOut(normal_arrow),
                FadeOut(normal_label),
                FadeOut(relative_arrow),
                FadeOut(relative_label),
                FadeIn(tangent_arrow),
                FadeIn(tangent_label),
                FadeIn(tangent_eq),
            )

            # ============================================================
            # APF REPULSIVE DISPLACEMENT
            # ============================================================

            apf_eq = MathTex(
                r"f_{ij}"
                r"=(d_{\min}-d_{ij})"
                r"\frac{\Delta_{ij}^{tan}}"
                r"{\|\Delta_{ij}^{tan}\|}",
                font_size=28
            ).next_to(tangent_eq, DOWN, buff=0.18)

            # Visualization scale
            d_min = 1.5
            repulsion_dir = tangent / np.linalg.norm(tangent)
            repulsion_force = np.linalg.norm(relative) - d_min
       
            repulsion_arrow = Arrow3D(
                start = q_i,
                end = q_i + repulsion_force*repulsion_dir,
                thickness=0.035,
                height=0.19,
                base_radius=0.07,
            )

            repulsion_label = MathTex(
                r"f_{ij}",
                font_size=27
            )

            repulsion_label_point = self.camera.project_point(q_i + repulsion_force*repulsion_dir)

            repulsion_label.move_to(repulsion_label_point + UP * 0.35)

            self.add_fixed_in_frame_mobjects(
                apf_eq,
                repulsion_label,
            )

            self.play(
                FadeOut(tangent_arrow),
                FadeOut(tangent_label),
                FadeIn(repulsion_arrow),
                FadeIn(repulsion_label),
                FadeIn(apf_eq),
            )

            # ============================================================
            # TOTAL APF DISPLACEMENT
            # ============================================================

            total_eq = MathTex(
                r"f_{\mathrm{total}}"
                r"=\sum_{j\neq i}f_{ij}",
                font_size=29
            ).next_to(apf_eq, DOWN, buff=0.18)

            self.add_fixed_in_frame_mobjects(total_eq)

            self.play(
                FadeIn(total_eq),
            )

            # ============================================================
            # RE-PROJECTION
            # ============================================================

            new_qi = q_i + repulsion_force * repulsion_dir

            # First move UAV to the unconstrained position
            label_i_point = self.camera.project_point(new_qi) 
            self.play(
                uav_i.animate.move_to(new_qi),
                label_i.animate.move_to(label_i_point + UP * 0.35),
            )

            new_qi -= AXIS_ORIGIN
            # Project back onto the sphere
            new_qi_projected = (
                R * ((new_qi) / np.linalg.norm(new_qi))
            )
            new_qi_projected += AXIS_ORIGIN

            proj_eq_1 = MathTex(
                r"\Pi_{\Gamma^2}^{1,0}(p_i) = \frac{p}{\|p\|}",
                font_size=28
            ).next_to(total_eq, DOWN, buff=0.18)
            self.add_fixed_in_frame_mobjects(proj_eq_1)

            # Move UAV back onto sphere
            label_i_point = self.camera.project_point(new_qi_projected)    
            self.play(
                uav_i.animate.move_to(new_qi_projected),
                label_i.animate.move_to(label_i_point + UP * 0.35),
                FadeIn(proj_eq_1),
                FadeOut(repulsion_arrow),
                FadeOut(repulsion_label),
                FadeOut(geodesic_label),
                FadeOut(geodesic),
            )

            self.next_slide()

            self.play(
                *[FadeOut(mob)for mob in self.mobjects]
            )

            # ----------------------------------------------------------
            # Add results plot
            # ----------------------------------------------------------
            screen_width = config.frame_width

            APF_test_top_inactive = ImageMobject("assets/images/APF_test_top_inactive.png")
            APF_test_top_inactive.set_y(0)
            APF_test_top_inactive.set_x(-screen_width/4)
            APF_test_top_inactive.set(width=6)
        
            APF_test_top_active = ImageMobject("assets/images/APF_test_top_active.png")
            APF_test_top_active.set_y(0)
            APF_test_top_active.set_x(screen_width/4)
            APF_test_top_active.set(width=6)

            self.add_fixed_in_frame_mobjects(
                APF_test_top_inactive,
                APF_test_top_active
            )

            self.play(
                FadeIn(APF_test_top_inactive),
                FadeIn(APF_test_top_active)
            )

            self.wait(5)

            self.next_slide()

    class Radial_control(Slide):
        def construct(self):
            # ============================================================
            # Helper: create the split-screen layout
            # ============================================================

            divider = Line(
                start=UP * 3.8,
                end=DOWN * 3.8,
                stroke_width=2,
            ).shift(RIGHT * 2.0)

            screen_width = config.frame_width
            line_x = divider.get_x()

            stored_title_middle_x = (line_x + screen_width / 2) / 2
    
            stored_title = Tex(
                r"\textbf{Stored equations}",
                font_size=30,
                color=YELLOW
            )
            stored_title.set_x(stored_title_middle_x)  
            stored_title.shift(UP * 3.1)

            active_title = Tex(
                r"\textbf{Current derivation}",
                font_size=30,
                color=YELLOW
            )

            active_title_middle_x = (config.frame_width / -2 + divider.get_x()) / 2
            active_title.set_x(active_title_middle_x)  
            active_title.shift(UP * 3.1)

            def show_split_screen():
                self.play(
                    Create(divider),
                    FadeIn(active_title),
                    FadeIn(stored_title),
                )

            stored_y = stored_title.get_bottom()[1] - 0.35
            stored_x = stored_title.get_bottom()[0]
            def store_equation(eq, buff=0.35):
                nonlocal stored_y

                eq_copy = eq.copy()
                eq_copy.scale(0.65)

                # Fixed horizontal position
                eq_copy.set_x(stored_x)

                # Put it at the current vertical position
                eq_copy.set_y(stored_y)

                self.play(FadeIn(eq_copy))

                # Move downward for the next equation
                stored_y -= eq_copy.height + buff

                return eq_copy

            # ------------------------------------------------------------
            # 1. Radial update
            # ------------------------------------------------------------

            dynamics = MathTex(
                r"\dot{q}_i = -k_{geo} S(e_{q_i}) q_i \text{ (11)}",
                font_size=38
            ).to_corner(UR, buff=0.5)

            r_dot = MathTex(
                r"""\dot{r}_i = k_{\mathrm{rad}} e^{-( \|e_{q_i}\|)^2}
                    \left( r^\star - r_i \right) \text{ (15)}""",
                font_size=38
            )

            proposition_2 = Tex(
                r"""\textbf{Proposition:} Consider the closed-loop system composed 
                of the reduced-attitude dynamics (11) and 
                the radial dynamics (15) for all 
                $i = 1,\dots,N$, where $k_{\mathrm{geo}} > 0$, $k_{\mathrm{rad}} > 0$, $r^\star > 0$ 
                is constant, and $e^{-\|e_{q_i}\|^2} > 0$. 
                Then, the equilibrium defined by
            
                $$\hat q_i = \hat q_i^\star, \qquad r_i = r^\star, $$
            
                is almost globally asymptotically stable.""",
                font_size=36,
                color=WHITE,
                tex_to_color_map={
                    r"\textbf{Proposition:}": YELLOW
                }
            ).shift(DOWN)

            dynamics.set_x(-screen_width / 4)
            dynamics.set_y(2.5)

            r_dot.set_x(screen_width / 4)
            r_dot.set_y(2.5)

            # Show the first two equations
            self.play(
                FadeIn(dynamics), 
                FadeIn(r_dot), 
                FadeIn(proposition_2)
            )

            # ============================================================
            # START SPLIT SCREEN
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
            architecture.to_corner(DL, buff=0.3)

            self.play(
                FadeOut(dynamics), 
                FadeOut(r_dot), 
                FadeOut(proposition_2),
                FadeIn(architecture),
                radial.animate.scale(1.35),
            )

            show_split_screen()
        
            # Store the equations already established
            stored_dynamics = store_equation(dynamics)
            stored_r_dot = store_equation(r_dot)

            # ============================================================
            # 2. Radial error dynamics
            # ============================================================
            er_eq = MathTex(
                "e_{r_i} = r_i -r^\star",
                font_size=40
            )
            er_eq.set_x(active_title_middle_x)
            er_eq.next_to(active_title, DOWN, buff = 0.18)

            self.play(FadeIn(er_eq))
            stored_er_eq = store_equation(er_eq)

            dot_er_equal = MathTex(
                "=",
                font_size=40
            )

            lhs = MathTex(
                "e_{r_i}",
                font_size=40
            ).next_to(dot_er_equal, LEFT) 

            rhs = MathTex(
                "r_i", "-r^\star",
                font_size=40
            ).next_to(dot_er_equal, RIGHT)


            dot_er_eq = VGroup(lhs, dot_er_equal, rhs).arrange(RIGHT, buff=0.15)
            dot_er_eq.move_to(er_eq)

            self.play(
                FadeOut(er_eq),
                FadeIn(dot_er_eq)
            )

            new_lhs = MathTex(
                r"\dot e_{r_i}",
                font_size=40
            ).next_to(dot_er_equal, LEFT) 

            new_rhs = MathTex(
                r"\dot r_i",
                font_size=40
            ).next_to(dot_er_equal, RIGHT)

            self.play(
                TransformMatchingTex(
                    lhs,
                    new_lhs,
                    run_time = 3
                ),
                TransformMatchingTex(
                    rhs,
                    new_rhs,
                    run_time = 3
                ),
            )
            lhs = new_lhs
            rhs = new_rhs

            self.play(
                Indicate(rhs),
                Indicate(stored_r_dot)
            )

            new_rhs = MathTex(
                r"k_{\mathrm{rad}} e^{-( \|e_{q_i}\|)^2} \left( r^\star - r_i \right)",
                font_size=40
            ).next_to(dot_er_equal, RIGHT)

            self.play(
                TransformMatchingTex(
                    rhs,
                    new_rhs,
                    run_time = 3
                ),
            )
            rhs_1 = new_rhs

            rhs = MathTex(
                r"k_{\mathrm{rad}} e^{-( \|e_{q_i}\|)^2}", r"\left( r^\star - r_i \right)",
                font_size=40
            ).next_to(dot_er_equal, RIGHT)

            new_rhs = MathTex(
                r"k_{\mathrm{rad}} e^{-( \|e_{q_i}\|)^2}", r"(-e_{r_i})",
                font_size=40
            ).next_to(dot_er_equal, RIGHT)

            self.play(
                Indicate(stored_er_eq),
                Indicate(rhs[1]),
            )

            self.play(
                FadeOut(rhs_1),
                TransformMatchingTex(
                    rhs,
                    new_rhs,
                    run_time = 3
                ),
            )
            rhs = new_rhs

            dot_er_eq = VGroup(lhs, dot_er_equal, rhs).arrange(RIGHT, buff=0.15)
            dot_er_eq.next_to(active_title, DOWN, buff=0.18)
            dot_er_eq.set_x(active_title_middle_x)

            # Build the final layout
            stored_dot_er_eq = store_equation(dot_er_eq)

            # ============================================================
            # 3. Composite Lyapunov function
            # ============================================================
            self.play(
                FadeOut(dot_er_eq)
            )

            V_equal = MathTex(
                r"=",
                font_size=40
            )

            lhs = MathTex(
                r"V",
                font_size=40
            ).next_to(V_equal, LEFT) 

            rhs = MathTex(
                r"\sum_{i=1}^{N} \left(\Psi_i+ \frac{1}{2} e_{r_i}^{2}\right)",
                font_size=40
            ).next_to(V_equal, RIGHT)

            V_eq = VGroup(lhs, V_equal, rhs).arrange(RIGHT, buff=0.15)
            V_eq.set_x(active_title_middle_x)
            V_eq.shift(UP*2)
        
            V_props = MathTex(
                r"""
                    V \geq 0, \quad V=0 \iff (e_{q_i}=0, e_{r_i}=0)
                """,
                font_size=32
            ).next_to(V_eq, DOWN, buff=0.55)

            self.play(
                FadeIn(V_eq),
                FadeIn(V_props)
            )

            self.next_slide()

            # ------------------------------------------------------------
            # 3. Lyapunov derivative
            # ------------------------------------------------------------

            self.play(
                FadeOut(V_props)
            )

            dot_V_equal = MathTex(
                r"=",
                font_size=40
            )

            lhs = MathTex(
                r"\dot V",
                font_size=40
            ).next_to(dot_V_equal, LEFT) 

            rhs = MathTex(
                r"\sum_i \left(", r"\dot \Psi_i", r"+ e_{r_i} \dot e_{r_i} \right)",
                font_size=40
            ).next_to(dot_V_equal, RIGHT)

            dot_V_eq = VGroup(lhs, dot_V_equal, rhs)
            dot_V_eq.next_to(V_eq, DOWN, buff=0.55)

            self.play(FadeIn(dot_V_eq))
            self.next_slide()

            new_rhs = MathTex(
                r"\sum_i \left(", r"-k_{\mathrm{geo}}\|e_{q_i}\|^2", r"+ e_{r_i} \dot e_{r_i} \right)",
                font_size=40
            ).next_to(dot_V_equal, RIGHT)

            self.play(
                TransformMatchingTex(
                    rhs,
                    new_rhs,
                    run_time = 3,
                ),
            )
            rhs = new_rhs

            dot_V_eq = VGroup(lhs, dot_V_equal, rhs)
            dot_V_eq.next_to(V_eq, DOWN, buff=0.55)

            self.play(dot_V_eq.animate.shift(LEFT*1.5))

            new_rhs = MathTex(
                r"\sum_i \left(", r"-k_{\mathrm{geo}}\|e_{q_i}\|^2", r"+ -k_{\mathrm{rad}} e^{-\|e_{q_i}\|^2} e_{r_i}^2 \right)",
                font_size=40
            ).next_to(dot_V_equal, RIGHT)
        

            self.play(
                TransformMatchingTex(
                    rhs,
                    new_rhs,
                    run_time = 3,
                ),
                Indicate(stored_dot_er_eq)
            )
            rhs = new_rhs

            new_rhs = MathTex(
                r"""
                    -\sum_i
                    k_{\mathrm{geo}}\|e_{q_i}\|^2
                    -
                    \sum_i
                    k_{\mathrm{rad}}
                    e^{-\|e_{q_i}\|^2}
                    e_{r_i}^2
                    \le 0
                """,
                font_size=40
            ).next_to(dot_V_equal, RIGHT)
    
            self.play(
                TransformMatchingTex(
                    rhs,
                    new_rhs,
                    run_time = 3,
                ),
            )
            rhs = new_rhs

            dot_V_results = MathTex(
                r"\left( k_{geo}>0, \quad k_{geo}>0, \quad e^{-\|e_{q_i}\|^2}>0  \right) \forall e_{q_i} \Longrightarrow \dot V \le 0",
                font_size=30
            ).next_to(dot_V_eq, DOWN, buff=0.55)
            dot_V_results.set_x(active_title_middle_x)

            self.play(
                FadeIn(dot_V_results)
            )

            self.next_slide()

    class Results(Slide):
        def construct(self):
            IMAGE_SCALE = 0.6
            # ===================== Simulation Results =====================
            # 3-drone results
            title = Text("3-Drone Scenario", font_size=40).to_edge(UP)

            img1 = ImageMobject(
                "assets/images/solutions/radial_hist_3_agents.png"
            ).scale(IMAGE_SCALE)
            img2 = ImageMobject(
                "assets/images/solutions/centralized_apf_30t_3Drones.png"
            ).scale(IMAGE_SCALE)
            img3 = ImageMobject(
                "assets/images/solutions/composed_3_agents.png"
            ).scale(IMAGE_SCALE)
            img4 = ImageMobject(
                "assets/images/solutions/intra_agent_dist_apf_3.png"
            ).scale(IMAGE_SCALE)

            images = Group(img1, img2, img3, img4).arrange_in_grid(
                rows=2, cols=2, buff=0.2
            ).next_to(title, DOWN)

            self.play(
                FadeIn(title),
                FadeIn(images)
            )
            self.next_slide()

            # 31-drone results
            self.play(
                FadeOut(title),
                FadeOut(images)
            )

            title = Text("31-Drone Scenario", font_size=40).to_edge(UP)

            img1 = ImageMobject(
                "assets/images/solutions/radial_hist_31_agents.png"
            ).scale(IMAGE_SCALE)
            img2 = ImageMobject(
                "assets/images/solutions/centralized_apf_30t_31Drones.png"
            ).scale(IMAGE_SCALE)
            img3 = ImageMobject(
                "assets/images/solutions/composed_31_agents.png"
            ).scale(IMAGE_SCALE)
            img4 = ImageMobject(
                "assets/images/solutions/intra_agent_dist_apf_31.png"
            ).scale(IMAGE_SCALE)

            images = Group(img1, img2, img3, img4).arrange_in_grid(
                rows=2, cols=2, buff=0.2
            ).next_to(title, DOWN)

            self.play(
                FadeIn(title),
                FadeIn(images)
            )
            self.next_slide()

            # Simulation parameters
            self.play(
                FadeOut(title),
                FadeOut(images)
            )
            self.next_slide()