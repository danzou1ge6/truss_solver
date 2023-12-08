use crate::solver::*;

use fltk::prelude::*;
use num_traits::Zero;
use std::cell::RefCell;
use std::rc::Rc;
use {
    fltk::app as fa, fltk::button as fb, fltk::draw as fd, fltk::enums as fe, fltk::frame as ff,
    fltk::group as fg, fltk::input as fi, fltk::menu as fm, fltk::output as fo,
    fltk::valuator as fv, fltk::widget as fw,
};

const MOUSE_LEFT: i32 = 1;
const MOUSE_RIGHT: i32 = 3;

#[derive(Debug, Clone)]
enum Stage {
    Idle,
    SelectedRod(NodeId, NodeId),
    SelectedNode(NodeId),
    SelectedNodeToDraw(NodeId),
    DraggingNode(NodeId),
    LeftPushedNode(NodeId),
    Moving,
}

#[derive(Debug, Clone)]
enum Message {
    Standby,
    AlreadyARodHere,
    InvalidFloat(&'static str),
    TrussSolve(TrussSolveError),
    IdleHint,
    DrawHint,
    SolveSuccess,
    UnacceptableBigDu,
}

impl std::fmt::Display for Message {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        use Message::*;
        match self {
            Standby => f.write_str("Standby"),
            AlreadyARodHere => f.write_str("Error: Already a rod is placed here"),
            InvalidFloat(name) => write!(f, "Error: Check your float input of {}", name),
            TrussSolve(e) => e.fmt(f),
            IdleHint => f.write_str("Left click to edit, right click to draw"),
            DrawHint => f.write_str("Left click to exit draw, right click to draw rod"),
            SolveSuccess => f.write_str("Successfully solved"),
            UnacceptableBigDu => {
                f.write_str("DU is unacceptably large on some nodes, structure may be unstable")
            }
        }
    }
}

pub struct State {
    truss: Truss,
    stage: Stage,
    msg: Message,
    changed: bool,
    truss_changed: bool,
    rod: Rod,
    rods: Vec<Rod>,
    show_numbers: bool,
    show_du: bool,
    auto_calc: bool,
    zoom: f64,
    mouse_pos: Vec2D,
    forces: Option<Forces>,
    du: Option<Displacement>,
    show_grid: bool,
}

impl Default for State {
    fn default() -> Self {
        State {
            truss: Truss::new(),
            stage: Stage::Idle,
            msg: Message::Standby,
            changed: false,
            truss_changed: false,
            show_du: false,
            rod: Rod::default(),
            rods: vec![Rod::default()],
            show_numbers: false,
            auto_calc: true,
            zoom: 10.0,
            mouse_pos: Vec2D::new(0.0, 0.0),
            forces: None,
            du: None,
            show_grid: true,
        }
    }
}

impl State {
    fn mark_changed(&mut self) {
        self.changed = true;
    }
    fn mark_truss_changed(&mut self) {
        self.truss_changed = true;
        self.forces = None;
    }
    fn stage_to(&mut self, stg: Stage) {
        match stg {
            Stage::Idle => self.msg = Message::IdleHint,
            Stage::SelectedNodeToDraw(..) => self.msg = Message::DrawHint,
            _ => {}
        }
        self.stage = stg;
        self.mark_changed();
    }
    fn set_msg(&mut self, msg: Message) {
        self.msg = msg;
        self.mark_changed();
    }
    fn calc_forces(&mut self) {
        if !self.truss_changed {
            return;
        }

        match self.truss.solve() {
            Ok(du) => {
                self.forces = Some(self.truss.rods_force(&du));
                if self
                    .truss
                    .nodes
                    .keys()
                    .copied()
                    .map(|i| {
                        du.get(i).unwrap().length()
                            > self
                                .truss
                                .neighbour_distances(i)
                                .unwrap()
                                .reduce(|a, b| a.min(b))
                                .unwrap_or(f64::MAX)
                                / 100.0
                    })
                    .any(|x| x)
                {
                    self.set_msg(Message::UnacceptableBigDu);
                } else {
                    self.set_msg(Message::SolveSuccess);
                }

                self.du = Some(du);
            }
            Err(e) => {
                self.set_msg(Message::TrussSolve(e));
                self.forces = None;
            }
        }
        self.truss_changed = false;
    }
    fn update_rod_types(&mut self) {
        let mut rod_set: std::collections::HashSet<Rod> = self.truss.rods().cloned().collect();
        rod_set.insert(self.rod.clone());
        self.rods = rod_set.into_iter().collect();
        self.mark_changed();
    }
    pub fn consume_change_flag(&mut self) -> bool {
        let r = self.changed;
        self.changed = false;
        return r;
    }
}

pub struct SegmentedColorMap<const N: usize> {
    segs: [(f64, u8, u8, u8); N],
}

impl<const N: usize> SegmentedColorMap<N> {
    pub const fn new(segs: [(f64, u8, u8, u8); N]) -> Self {
        Self { segs }
    }
    pub fn color(&self, x: f64) -> (u8, u8, u8) {
        let upper = self
            .segs
            .iter()
            .position(|(a, ..)| *a > x)
            .expect("x should be within [0, 1]");
        let (b, r2, g2, b2) = self.segs[upper];
        let (a, r1, g1, b1) = self.segs[upper - 1];

        let map = |lo: u8, hi: u8| -> u8 {
            let r = (x - a) / (b - a) * (f64::from(hi) - f64::from(lo)) + f64::from(lo);
            r as u8
        };

        (map(r1, r2), map(g1, g2), map(b1, b2))
    }
}

const COLOR_MAP: SegmentedColorMap<5> = SegmentedColorMap::new([
    (0.0, 63, 51, 255),
    (0.25, 51, 233, 255),
    (0.5, 51, 255, 51),
    (0.75, 255, 252, 51),
    (1.00001, 255, 87, 51),
]);

fn linear_normalize<T>(lo: T, hi: T, x: T) -> T
where
    T: std::ops::Sub<T, Output = T> + std::ops::Div<T, Output = T> + Copy,
{
    (x - lo) / (hi - lo)
}

fn linear_map<T>(lo: T, hi: T, x: T, to_lo: T, to_hi: T) -> T
where
    T: Copy + num_traits::Num,
{
    (x - lo) / (hi - lo) * (to_hi - to_lo) + to_lo
}

struct Canvas {
    inner: fw::Widget,
}

struct CanvasState {
    gstate: Rc<RefCell<State>>,
    top_left: Vec2D,
    zoom: f64,
    mouse_pos: (i32, i32),
    h: i32,
    w: i32,
    rounding_length: f64,
}

fn smart_format_float(x: f64) -> String {
    if 0.1 <= x.abs() && x.abs() < 100.0 {
        format!("{:.2}", x)
    } else if x.abs() < 1e-5 {
        format!("0")
    } else {
        format!("{:.2e}", x)
    }
}

impl CanvasState {
    fn canvas_to_screen(&self, pos: &Vec2D) -> (i32, i32) {
        (
            ((pos.x - self.top_left.x) * self.zoom) as i32,
            ((self.top_left.y - pos.y) * self.zoom) as i32,
        )
    }

    fn screen_to_canvas(&self, u: i32, v: i32) -> Vec2D {
        Vec2D::new(
            self.top_left.x + (u as f64) / self.zoom,
            self.top_left.y - (v as f64) / self.zoom,
        )
    }

    const NODE_CANVAS_RADIUS: f64 = 0.4;

    fn node_screen_radius(&self) -> i32 {
        (Self::NODE_CANVAS_RADIUS * self.zoom) as i32
    }

    fn draw_line_canvas(&self, a: Vec2D, b: Vec2D) {
        let (u1, v1) = self.canvas_to_screen(&a);
        let (u2, v2) = self.canvas_to_screen(&b);
        fd::draw_line(u1, v1, u2, v2);
    }

    fn draw_wall(&self, center: Vec2D, phi: f64) {
        let n = Vec2D::new_polar(phi);
        let d = Vec2D::new(-phi.sin(), phi.cos());

        let mid = center - Self::NODE_CANVAS_RADIUS * n;
        let left = mid - 4.0 * Self::NODE_CANVAS_RADIUS * d;
        let right = mid + 4.0 * Self::NODE_CANVAS_RADIUS * d;

        fd::set_draw_color(fe::Color::Dark2);
        fd::set_line_style(fd::LineStyle::Solid, 2);
        self.draw_line_canvas(left, right);

        for i in -3..=3 {
            let top = mid + Self::NODE_CANVAS_RADIUS * f64::from(i) * d;
            let bottom =
                top - Self::NODE_CANVAS_RADIUS * Vec2D::new_polar(phi + std::f64::consts::PI / 5.0);
            self.draw_line_canvas(top, bottom);
        }
    }

    fn draw_force(&self, at: Vec2D, phi: f64, force: f64) {
        let n = Vec2D::new_polar(phi);
        let begin = at - Self::NODE_CANVAS_RADIUS * 8.0 * n;
        let end = at - Self::NODE_CANVAS_RADIUS * n;

        let cw_begin = end
            - Self::NODE_CANVAS_RADIUS * 2.0 * Vec2D::new_polar(phi + std::f64::consts::PI / 6.0);
        let ccw_begin = end
            - Self::NODE_CANVAS_RADIUS * 2.0 * Vec2D::new_polar(phi - std::f64::consts::PI / 6.0);

        if self.gstate.borrow().show_numbers {
            let (u, v) = self.canvas_to_screen(&begin);
            fd::set_draw_color(fe::Color::Black);
            fd::draw_text(&&smart_format_float(force), u, v);
        }

        fd::set_draw_color(fe::Color::Dark2);
        fd::set_line_style(fd::LineStyle::Solid, 2);
        self.draw_line_canvas(cw_begin, end);
        self.draw_line_canvas(ccw_begin, end);
        self.draw_line_canvas(begin, end);
    }

    fn draw_node_du(&self, at: Vec2D, value: Vec2D, node: NodeId) {
        let n = value.normalized();
        let end = at + Self::NODE_CANVAS_RADIUS * 8.0 * n;
        let begin = at + Self::NODE_CANVAS_RADIUS * n;

        let cw_begin = end
            - Self::NODE_CANVAS_RADIUS
                * 2.0
                * n.counter_clockwise_rotated(std::f64::consts::PI / 6.0);
        let ccw_begin = end
            - Self::NODE_CANVAS_RADIUS
                * 2.0
                * n.counter_clockwise_rotated(-std::f64::consts::PI / 6.0);

        if self.gstate.borrow().show_numbers {
            let (u, v) = self.canvas_to_screen(&end);
            fd::set_draw_color(fe::Color::Black);
            fd::draw_text(&&smart_format_float(value.length()), u, v);
        }

        if value.length()
            > self
                .gstate
                .borrow()
                .truss
                .neighbour_distances(node)
                .unwrap()
                .reduce(|a, b| a.min(b))
                .unwrap_or(f64::MAX)
                / 100.0
        {
            fd::set_draw_color(fe::Color::Red);
        } else {
            fd::set_draw_color(fe::Color::Dark2);
        }
        fd::set_line_style(fd::LineStyle::Dash, 2);
        self.draw_line_canvas(cw_begin, end);
        self.draw_line_canvas(ccw_begin, end);
        self.draw_line_canvas(begin, end);
    }

    fn node_screen_pos(&self, i: &NodeId) -> (i32, i32) {
        self.canvas_to_screen(&self.gstate.borrow().truss.nodes[i].pos)
    }

    fn draw_color_bar(&self) {
        if let Some(forces) = &self.gstate.borrow().forces {
            let max = forces.abs_max().unwrap_or(1.0);

            let top = self.h / 4;
            let bottom = self.h * 3 / 4;

            for y in top..bottom {
                let (r, g, b) = COLOR_MAP.color(linear_normalize(
                    f64::from(bottom),
                    f64::from(top),
                    f64::from(y),
                ));
                fd::set_draw_rgb_color(r, g, b);
                fd::set_line_style(fd::LineStyle::Solid, 1);
                fd::draw_xyline(0, y, 5);
            }

            for y in (top..bottom).step_by(40) {
                let value = linear_map(f64::from(bottom), f64::from(top), f64::from(y), -max, max);
                fd::set_draw_color(fe::Color::Black);
                fd::draw_text(&smart_format_float(value), 5, y);
            }
        }
    }

    fn draw_rods(&self) {
        let state_borrow = self.gstate.borrow();

        for (i, edges) in state_borrow.truss.graph.iter() {
            for (j, _) in edges.iter().filter(|(j, _)| **j < *i) {
                let (ui, vi) = self.node_screen_pos(i);
                let (uj, vj) = self.node_screen_pos(j);

                if let Some(forces) = &self.gstate.borrow().forces {
                    let max = forces.abs_max().unwrap_or(1.0);
                    let f = forces.get(*i, *j).unwrap();

                    let color_x = if max > 0.0 {
                        linear_normalize(-max, max, f)
                    } else {
                        0.0
                    };

                    let (r, g, b) = COLOR_MAP.color(color_x);
                    fd::set_color_rgb(r, g, b);
                    fd::set_line_style(fd::LineStyle::Solid, 4);
                    fd::draw_line(ui, vi, uj, vj);

                    if self.gstate.borrow().show_numbers {
                        let us = (ui + uj) / 2;
                        let js = (vi + vj) / 2;
                        fd::set_draw_color(fe::Color::Black);
                        fd::draw_text(&smart_format_float(f), us, js);
                    }
                } else {
                    fd::set_draw_color(fe::Color::Dark2);
                    fd::set_line_style(fd::LineStyle::Solid, 4);
                    fd::draw_line(ui, vi, uj, vj);
                }
                fd::set_line_style(fd::LineStyle::Solid, 2);
            }
        }
    }

    fn draw_chart(&self) {
        if self.gstate.borrow().show_grid {
            let right_bottom0 = self.screen_to_canvas(self.w, self.h);
            let right_bottom = self.round_pos(right_bottom0);
            let left_top = self.round_pos(self.top_left);

            fd::set_draw_color(fe::Color::Light2);
            fd::set_line_style(fd::LineStyle::Solid, 2);

            let mut x = left_top.x;
            while x <= right_bottom.x {
                self.draw_line_canvas(
                    Vec2D::new(x, self.top_left.y),
                    Vec2D::new(x, right_bottom0.y),
                );
                x += self.rounding_length;
            }

            let mut y = right_bottom.y;
            while y <= left_top.y {
                self.draw_line_canvas(
                    Vec2D::new(self.top_left.x, y),
                    Vec2D::new(right_bottom0.x, y),
                );
                y += self.rounding_length;
            }
        }
    }

    fn draw_rod_xy(&self, a: Vec2D, b: Vec2D) {
        let (ua, va) = self.canvas_to_screen(&a);
        let (ub, vb) = self.canvas_to_screen(&b);
        let us = (ua + ub) / 2;
        let vs = (va + vb) / 2;

        fd::set_draw_color(fe::Color::Dark2);
        fd::set_line_style(fd::LineStyle::Dot, 2);
        fd::draw_line(ua, va, ub, va);
        fd::draw_line(ub, va, ub, vb);

        if !(a.x - b.x).is_zero() {
            fd::draw_text(&smart_format_float((a.x - b.x).abs()), us, va);
        }

        if !(a.y - b.y).is_zero() {
            fd::draw_text(&smart_format_float((a.y - b.y).abs()), ub, vs);
        }
    }

    fn draw_du(&self) {
        if !self.gstate.borrow().show_du {
            return;
        }

        if let Some(du) = &self.gstate.borrow().du {
            for (id, node) in self.gstate.borrow().truss.nodes.iter() {
                if let Some(displacement) = du.get(*id) {
                    self.draw_node_du(node.pos, displacement, *id);
                }
            }
        }
    }

    fn draw(&self) {
        if self.gstate.borrow().auto_calc {
            self.gstate.borrow_mut().calc_forces();
        }

        fd::draw_rect_fill(0, 0, self.w, self.h, fe::Color::White);

        self.draw_chart();

        self.draw_rods();
        self.draw_color_bar();

        for node in self.gstate.borrow().truss.nodes.values() {
            let (u, v) = self.canvas_to_screen(&node.pos);
            fd::draw_circle_fill(
                u - self.node_screen_radius(),
                v - self.node_screen_radius(),
                self.node_screen_radius() * 2,
                fe::Color::Dark2,
            );

            match &node.constrain {
                Constrain::Force { phi, f } => self.draw_force(node.pos, *phi, *f),
                Constrain::Hinge => {
                    use std::f64::consts::PI;
                    for phi in [0.0, 0.5 * PI, PI, 1.5 * PI] {
                        self.draw_wall(node.pos, phi);
                    }
                }
                Constrain::Slide(phi) => {
                    self.draw_wall(node.pos, *phi);
                    self.draw_wall(node.pos, *phi + std::f64::consts::PI);
                }
                Constrain::None => {}
            }
        }

        self.draw_du();

        let stage = self.gstate.borrow().stage.clone();
        match stage {
            Stage::SelectedRod(i, j) => {
                let (ui, vi) = self.node_screen_pos(&i);
                let (uj, vj) = self.node_screen_pos(&j);

                fd::set_draw_color(fe::Color::White);
                fd::set_line_style(fd::LineStyle::Solid, 4);
                fd::draw_line(ui, vi, uj, vj);

                fd::set_draw_color(fe::Color::Black);
                fd::set_line_style(fd::LineStyle::DashDot, 4);
                fd::draw_line(ui, vi, uj, vj);

                let posi = self.gstate.borrow_mut().truss.nodes[&i].pos;
                let posj = self.gstate.borrow_mut().truss.nodes[&j].pos;
                self.draw_rod_xy(posi, posj);

                fd::set_line_style(fd::LineStyle::Solid, 2);
            }
            Stage::SelectedNode(i) => {
                let (ui, vi) = self.node_screen_pos(&i);

                fd::draw_circle_fill(
                    ui - self.node_screen_radius(),
                    vi - self.node_screen_radius(),
                    self.node_screen_radius() * 2,
                    fe::Color::Black,
                );
            }
            Stage::SelectedNodeToDraw(i) => {
                let (ui, vi) = self.node_screen_pos(&i);
                let new_node = self.round_pos(self.mouse_canvas_pos());
                let (um, vm) = self.canvas_to_screen(&new_node);

                fd::set_draw_color(fe::Color::Dark2);
                fd::set_line_style(fd::LineStyle::Dash, 2);
                fd::draw_line(ui, vi, um, vm);

                let node = self.gstate.borrow().truss.nodes[&i].pos;
                self.draw_rod_xy(node, new_node);
            }
            Stage::DraggingNode(i) => {
                let posi = self.gstate.borrow_mut().truss.nodes[&i].pos;

                let neighbors = self.gstate.borrow_mut().truss.graph[&i]
                    .keys()
                    .copied()
                    .collect::<Vec<_>>();
                for j in neighbors.into_iter() {
                    let posj = self.gstate.borrow_mut().truss.nodes[&j].pos;
                    self.draw_rod_xy(posi, posj);
                }
            }
            _ => {}
        }
    }

    fn mouse_canvas_pos(&self) -> Vec2D {
        self.screen_to_canvas(self.mouse_pos.0, self.mouse_pos.1)
    }

    fn align_view(&mut self, canvas_pos: Vec2D, (screen_u, screen_v): (i32, i32)) {
        let p = self.screen_to_canvas(screen_u, screen_v);
        self.top_left += canvas_pos - p;
    }

    fn selection_radius(&self) -> f64 {
        0.5
    }

    fn round_pos(&self, pos: Vec2D) -> Vec2D {
        Vec2D::new(
            (pos.x / self.rounding_length).round() * self.rounding_length,
            (pos.y / self.rounding_length).round() * self.rounding_length,
        )
    }

    fn select(&mut self) {
        let mouse = self.mouse_canvas_pos();

        let option_i = self
            .gstate
            .borrow()
            .truss
            .node_by_pos(&mouse, self.selection_radius());
        if let Some(i) = option_i {
            self.gstate.borrow_mut().stage_to(Stage::SelectedNode(i));
            return;
        }

        let option_i = self
            .gstate
            .borrow()
            .truss
            .rod_by_pos(&mouse, self.selection_radius());
        if let Some((i, j)) = option_i {
            self.gstate.borrow_mut().stage_to(Stage::SelectedRod(i, j));
            return;
        }

        self.gstate.borrow_mut().stage_to(Stage::Idle);
    }

    fn select_to_add(&mut self) {
        let mouse = self.mouse_canvas_pos();

        if self.gstate.borrow().truss.nodes.is_empty() {
            self.gstate
                .borrow_mut()
                .truss
                .add_node(self.round_pos(mouse));
            self.gstate.borrow_mut().mark_changed();
            return;
        }

        let option_i = self
            .gstate
            .borrow()
            .truss
            .node_by_pos(&mouse, self.selection_radius());
        if let Some(i) = option_i {
            self.gstate
                .borrow_mut()
                .stage_to(Stage::SelectedNodeToDraw(i))
        }
    }

    fn add_rod(&mut self, from: NodeId) {
        let mouse = self.mouse_canvas_pos();

        let rod = self.gstate.borrow().rod.clone();

        let option_j = self
            .gstate
            .borrow()
            .truss
            .node_by_pos(&mouse, self.selection_radius());
        if let Some(j) = option_j {
            if self
                .gstate
                .borrow_mut()
                .truss
                .add_rod(from, j, rod)
                .is_err()
            {
                self.gstate.borrow_mut().set_msg(Message::AlreadyARodHere);
            }
            self.gstate.borrow_mut().mark_truss_changed();
            self.gstate.borrow_mut().stage_to(Stage::Idle);
        } else {
            let new_node = self
                .gstate
                .borrow_mut()
                .truss
                .add_node(self.round_pos(mouse));
            self.gstate
                .borrow_mut()
                .truss
                .add_rod(from, new_node, rod)
                .unwrap();
            self.gstate.borrow_mut().mark_truss_changed();
            self.gstate.borrow_mut().stage_to(Stage::Idle);
        }
    }

    fn cancel_select_to_draw(&mut self) {
        self.gstate.borrow_mut().stage_to(Stage::Idle);
    }

    fn release_stage_transition(&mut self) {
        let stage = self.gstate.borrow().stage.clone();
        match stage {
            Stage::Idle | Stage::LeftPushedNode(..) => match fa::event_button() {
                MOUSE_LEFT => self.select(),
                MOUSE_RIGHT => self.select_to_add(),
                _ => {}
            },
            Stage::SelectedNodeToDraw(i) => match fa::event_button() {
                MOUSE_RIGHT => self.add_rod(i),
                MOUSE_LEFT => self.cancel_select_to_draw(),
                _ => {}
            },
            Stage::SelectedRod(..) | Stage::SelectedNode(_) => {
                if fa::event_button() == MOUSE_LEFT {
                    self.select();
                }
            }
            Stage::Moving => {
                self.gstate.borrow_mut().stage_to(Stage::Idle);
            }
            Stage::DraggingNode(i) => {
                let mouse = self.round_pos(self.mouse_canvas_pos());
                let option_j = self.gstate.borrow().truss.node_by_pos_but_not(
                    &mouse,
                    self.selection_radius(),
                    i,
                );
                if let Some(j) = option_j {
                    self.gstate.borrow_mut().truss.merge_nodes(i, j).unwrap();
                    self.gstate.borrow_mut().mark_truss_changed();
                }

                self.gstate.borrow_mut().stage_to(Stage::Idle);
            }
        }
    }

    fn push_stage_transition(&mut self) {
        let stage = self.gstate.borrow().stage.clone();
        match stage {
            Stage::Idle if fa::event_button() == MOUSE_LEFT => {
                let mouse = self.mouse_canvas_pos();
                let option_i = self
                    .gstate
                    .borrow()
                    .truss
                    .node_by_pos(&mouse, self.selection_radius());
                if let Some(i) = option_i {
                    self.gstate.borrow_mut().stage_to(Stage::LeftPushedNode(i));
                    return;
                }
            }
            _ => {}
        }
    }

    fn drag_stage_transition(&mut self) {
        let new_u = fa::event_x();
        let new_v = fa::event_y();
        let du = new_u - self.mouse_pos.0;
        let dv = new_v - self.mouse_pos.1;

        let stage = self.gstate.borrow().stage.clone();
        match stage {
            Stage::Idle | Stage::Moving => {
                self.gstate.borrow_mut().stage_to(Stage::Moving);

                self.top_left.x -= f64::from(du) / self.zoom;
                self.top_left.y += f64::from(dv) / self.zoom;

                self.update_mouse_pos(new_u, new_v);
            }
            Stage::DraggingNode(i) | Stage::LeftPushedNode(i) => {
                self.gstate.borrow_mut().stage_to(Stage::DraggingNode(i));

                self.update_mouse_pos(new_u, new_v);

                let node_new_pos = self.round_pos(self.mouse_canvas_pos());
                self.gstate
                    .borrow_mut()
                    .truss
                    .nodes
                    .get_mut(&i)
                    .unwrap()
                    .pos = node_new_pos;
                self.gstate.borrow_mut().mark_truss_changed();
            }
            _ => {}
        }
    }

    fn update_mouse_pos(&mut self, u: i32, v: i32) {
        self.mouse_pos = (u, v);
        let pos = self.mouse_canvas_pos();
        self.gstate.borrow_mut().mouse_pos = pos;
        self.gstate.borrow_mut().mark_changed();
    }

    fn update_zoom(&mut self, factor: f64) {
        self.zoom *= factor;
        self.gstate.borrow_mut().zoom = self.zoom;
        self.rounding_length = 10.0_f64.powf(-self.zoom.log10().floor() + 1.0);
    }

    fn handle(&mut self, ev: fe::Event) -> bool {
        match ev {
            fe::Event::Enter | fe::Event::Leave => return true,
            fe::Event::Push => {
                self.push_stage_transition();
                return true;
            }
            fe::Event::Released => {
                self.release_stage_transition();
                return true;
            }
            fe::Event::Move => {
                self.update_mouse_pos(fa::event_x(), fa::event_y());
                return true;
            }
            fe::Event::Drag => {
                self.drag_stage_transition();
                return true;
            }
            fe::Event::MouseWheel => {
                let mouse_canvas_pos = self.mouse_canvas_pos();
                let scroll = match fa::event_dy() {
                    fa::MouseWheel::Up => -1,
                    fa::MouseWheel::Down => 1,
                    _ => 0,
                };
                self.update_zoom(1.1_f64.powi(scroll));
                self.gstate.borrow_mut().mark_changed();

                self.align_view(mouse_canvas_pos, self.mouse_pos);

                self.gstate.borrow_mut().mark_changed();
                return true;
            }
            _ => return false,
        }
    }
}

impl Canvas {
    pub fn new(w: i32, h: i32, gstate: Rc<RefCell<State>>) -> Self {
        let mut inner = fw::Widget::default().with_size(w, h);
        let cstate = CanvasState {
            gstate,
            top_left: Vec2D::new(-f64::from(w / 2), f64::from(h / 2)),
            zoom: 10.00001,
            mouse_pos: (0, 0),
            h,
            w,
            rounding_length: 1.0,
        };
        let cstate = Rc::new(RefCell::new(cstate));

        inner.draw({
            let cstate = cstate.clone();
            move |_| cstate.borrow().draw()
        });
        inner.handle({
            let cstate = cstate.clone();
            move |_, ev| cstate.borrow_mut().handle(ev)
        });

        Self { inner }
    }
}

#[derive(Clone)]
struct NodePropertyEditor {
    inner: fg::Group,
    gstate: Rc<RefCell<State>>,
    x_inp: fi::FloatInput,
    y_inp: fi::FloatInput,
    force_phi: fv::HorNiceSlider,
    slide_phi: fv::HorNiceSlider,
    force_inp: fi::FloatInput,
    force_group: fg::Group,
    slide_group: fg::Group,
    none_group: fg::Group,
    hinge_group: fg::Group,
    tabs: fg::Tabs,
}

impl NodePropertyEditor {
    pub fn new(w: i32, h: i32, gstate: Rc<RefCell<State>>) -> Self {
        let inner = fg::Group::default().with_size(w, h);

        let mut grid = fg::VGrid::new(0, 0, w, 2 * 40, "");
        grid.set_params(2, 2, 5);

        ff::Frame::default().with_size(0, 30).with_label("X = ");
        let mut x_inp = fi::FloatInput::default().with_size(0, 30);
        x_inp.set_callback({
            let gstate = gstate.clone();
            move |x_inp| {
                let stage = gstate.borrow().stage.clone();
                if let Stage::SelectedNode(i) = stage {
                    if let Ok(x) = x_inp.value().parse::<f64>() {
                        gstate.borrow_mut().truss.nodes.get_mut(&i).unwrap().pos.x = x;
                        gstate.borrow_mut().mark_truss_changed();
                        gstate.borrow_mut().mark_changed();
                    } else {
                        gstate.borrow_mut().set_msg(Message::InvalidFloat("X"))
                    }
                }
            }
        });

        ff::Frame::default().with_size(0, 30).with_label("Y = ");
        let mut y_inp = fi::FloatInput::default().with_size(0, 30);
        y_inp.set_callback({
            let gstate = gstate.clone();
            move |y_inp| {
                let stage = gstate.borrow().stage.clone();
                if let Stage::SelectedNode(i) = stage {
                    if let Ok(y) = y_inp.value().parse::<f64>() {
                        gstate.borrow_mut().truss.nodes.get_mut(&i).unwrap().pos.y = y;
                        gstate.borrow_mut().mark_truss_changed();
                        gstate.borrow_mut().mark_changed();
                    } else {
                        gstate.borrow_mut().set_msg(Message::InvalidFloat("Y"))
                    }
                }
            }
        });

        grid.end();

        let mut tabs = fg::Tabs::default().with_pos(0, 40).with_size(w, 40 * 4);

        tabs.set_callback({
            let gstate = gstate.clone();
            move |this| {
                let stage = gstate.borrow().stage.clone();
                if let Stage::SelectedNode(i) = stage {
                    {
                        let mut gstate_borrow = gstate.borrow_mut();
                        let node = gstate_borrow.truss.nodes.get_mut(&i).unwrap();
                        match &this.value().unwrap().label()[..] {
                            "None" => node.constrain = Constrain::None,
                            "Hinge" => node.constrain = Constrain::Hinge,
                            "Slide" => {
                                node.constrain = Constrain::Slide(std::f64::consts::PI / 2.0)
                            }
                            "Force" => {
                                node.constrain = Constrain::Force {
                                    f: 1.0,
                                    phi: -std::f64::consts::PI / 2.0,
                                }
                            }
                            _ => unreachable!(),
                        }
                    }
                    gstate.borrow_mut().mark_truss_changed();
                    gstate.borrow_mut().mark_changed();
                }
            }
        });

        let none_group = fg::Group::default().with_size(w, 4 * 40).with_label("None");
        ff::Frame::default()
            .with_size(100, 30)
            .below_of(&y_inp, 10)
            .with_label("      No Configurations");
        none_group.end();

        let hinge_group = fg::Group::default()
            .with_size(w, 4 * 40)
            .with_label("Hinge");
        ff::Frame::default()
            .with_size(100, 30)
            .below_of(&y_inp, 10)
            .with_label("      No Configurations");
        hinge_group.end();

        let slide_group = fg::Group::default()
            .with_size(w, 4 * 40)
            .with_label("Slide");
        let mut slide_phi = fv::HorNiceSlider::default()
            .with_size(w, 40)
            .below_of(&y_inp, 10)
            .with_label("Phi");
        slide_phi.set_bounds(0.0, std::f64::consts::PI);

        slide_phi.set_callback({
            let gstate = gstate.clone();
            move |this| {
                let stage = gstate.borrow().stage.clone();
                if let Stage::SelectedNode(i) = stage {
                    match &mut gstate
                        .borrow_mut()
                        .truss
                        .nodes
                        .get_mut(&i)
                        .unwrap()
                        .constrain
                    {
                        Constrain::Slide(phi) => {
                            *phi = this.value();
                        }
                        _ => {}
                    }
                    gstate.borrow_mut().mark_truss_changed();
                    gstate.borrow_mut().mark_changed();
                }
            }
        });
        slide_group.end();

        let force_group = fg::Group::default()
            .with_size(w, 4 * 40)
            .with_label("Force");

        let force_label = ff::Frame::default()
            .with_size(w / 2, 30)
            .below_of(&y_inp, 10)
            .with_label("F = ");
        let mut force_inp = fi::FloatInput::default()
            .with_size(w / 2, 30)
            .right_of(&force_label, 5);
        force_inp.set_callback({
            let gstate = gstate.clone();
            move |f_inp| {
                let stage = gstate.borrow().stage.clone();
                if let Stage::SelectedNode(i) = stage {
                    f_inp.value().parse::<f64>().map_or_else(
                        |_| {
                            gstate.borrow_mut().set_msg(Message::InvalidFloat("F"));
                        },
                        |f| {
                            match &mut gstate
                                .borrow_mut()
                                .truss
                                .nodes
                                .get_mut(&i)
                                .unwrap()
                                .constrain
                            {
                                Constrain::Force { f: ff, .. } => *ff = f,
                                _ => {}
                            };
                            gstate.borrow_mut().mark_truss_changed();
                            gstate.borrow_mut().mark_changed();
                        },
                    );
                }
            }
        });

        let mut force_phi = fv::HorNiceSlider::default()
            .with_size(w, 40)
            .below_of(&force_label, 5)
            .with_label("Phi");
        force_phi.set_bounds(-std::f64::consts::PI, std::f64::consts::PI);

        force_phi.set_callback({
            let gstate = gstate.clone();
            move |this| {
                let stage = gstate.borrow().stage.clone();
                if let Stage::SelectedNode(i) = stage {
                    match &mut gstate
                        .borrow_mut()
                        .truss
                        .nodes
                        .get_mut(&i)
                        .unwrap()
                        .constrain
                    {
                        Constrain::Force { phi, .. } => {
                            *phi = this.value();
                        }
                        _ => {}
                    }
                    gstate.borrow_mut().mark_truss_changed();
                    gstate.borrow_mut().mark_changed();
                }
            }
        });
        force_group.end();

        tabs.end();

        let mut remove_button = fb::Button::default()
            .with_size(80, 30)
            .with_label("Remove")
            .with_pos(0, h * 3 / 4);
        remove_button.set_callback({
            let gstate = gstate.clone();
            move |_| {
                let stage = gstate.borrow().stage.clone();
                if let Stage::SelectedNode(i) = stage {
                    gstate.borrow_mut().truss.remove_node(i).unwrap();
                    gstate.borrow_mut().mark_truss_changed();
                    gstate.borrow_mut().stage_to(Stage::Idle);
                    gstate.borrow_mut().update_rod_types();
                }
            }
        });

        inner.end();

        // inner
        // |- grid
        // |  |- x_inp
        // |  |- y_inp
        // |- tabs
        //    |- none_group
        //    |- hinge_group
        //    |- slide_group
        //       |- slide_phi
        //    |- force_group
        //       |- force_phi
        //       |- f_inp
        // |- remove_button

        Self {
            inner,
            gstate,
            x_inp,
            y_inp,
            force_inp,
            tabs,
            force_phi,
            slide_phi,
            slide_group,
            none_group,
            hinge_group,
            force_group,
        }
    }

    fn update(&mut self) {
        let stage = self.gstate.borrow().stage.clone();
        if let Stage::SelectedNode(i) = stage {
            self.x_inp
                .set_value(&(&self.gstate.borrow().truss.nodes[&i]).pos.x.to_string());
            self.y_inp
                .set_value(&(&self.gstate.borrow().truss.nodes[&i]).pos.y.to_string());

            let constrain = (&self.gstate.borrow().truss.nodes[&i]).constrain.clone();
            let tab_group = match constrain {
                Constrain::Force { phi, f } => {
                    self.force_phi.set_value(phi);
                    self.force_inp.set_value(&f.to_string());
                    &self.force_group
                }
                Constrain::Hinge => &self.hinge_group,
                Constrain::Slide(phi) => {
                    self.slide_phi.set_value(phi);
                    &self.slide_group
                }
                Constrain::None => &self.none_group,
            };

            let _ = self.tabs.set_value(tab_group);
        }
    }
}

#[derive(Clone)]
struct RodPropertyEditor {
    inner: fg::Group,
    elastic_inp: fi::FloatInput,
    area_inp: fi::FloatInput,
    length_output: fo::Output,
    stiffness_output: fo::Output,
    gstate: Rc<RefCell<State>>,
}

impl RodPropertyEditor {
    pub fn new(w: i32, h: i32, gstate: Rc<RefCell<State>>) -> Self {
        let inner = fg::Group::default().with_size(w, h);

        let mut grid = fg::VGrid::new(0, 0, w, 4 * 40, "");
        grid.set_params(4, 2, 5);

        ff::Frame::default().with_size(0, 30).with_label("E = ");
        let mut elastic_inp = fi::FloatInput::default().with_size(0, 30);
        elastic_inp.set_callback({
            let gstate = gstate.clone();
            move |elastic_inp| {
                let stage = gstate.borrow().stage.clone();
                if let Stage::SelectedRod(i, j) = stage {
                    if let Ok(e) = elastic_inp.value().parse::<f64>() {
                        gstate.borrow_mut().truss.rod_mut(i, j).unwrap().elastic = e;
                        gstate.borrow_mut().mark_truss_changed();
                        gstate.borrow_mut().update_rod_types();
                    } else {
                        gstate.borrow_mut().set_msg(Message::InvalidFloat("E"));
                    }
                }
            }
        });

        ff::Frame::default().with_size(0, 30).with_label("A = ");
        let mut area_inp = fi::FloatInput::default().with_size(0, 30);
        area_inp.set_callback({
            let gstate = gstate.clone();
            move |area_inp| {
                let stage = gstate.borrow().stage.clone();
                if let Stage::SelectedRod(i, j) = stage {
                    if let Ok(a) = area_inp.value().parse::<f64>() {
                        gstate.borrow_mut().truss.rod_mut(i, j).unwrap().area = a;
                        gstate.borrow_mut().mark_truss_changed();
                        gstate.borrow_mut().update_rod_types();
                    } else {
                        gstate.borrow_mut().set_msg(Message::InvalidFloat("A"));
                    }
                }
            }
        });

        ff::Frame::default().with_size(0, 30).with_label("L = ");
        let length_output = fo::Output::default().with_size(0, 30);

        ff::Frame::default().with_size(0, 30).with_label("k = ");
        let stiffness_output = fo::Output::default().with_size(0, 30);

        grid.end();

        let mut remove_button = fb::Button::default()
            .with_size(80, 30)
            .with_label("Remove")
            .below_of(&stiffness_output, 10);
        remove_button.set_callback({
            let gstate = gstate.clone();
            move |_| {
                let stage = gstate.borrow().stage.clone();
                if let Stage::SelectedRod(i, j) = stage {
                    gstate.borrow_mut().truss.remove_rod(i, j).unwrap();
                    gstate.borrow_mut().mark_truss_changed();
                    gstate.borrow_mut().stage_to(Stage::Idle);
                    gstate.borrow_mut().update_rod_types();
                }
            }
        });

        inner.end();

        Self {
            inner,
            elastic_inp,
            area_inp,
            gstate,
            stiffness_output,
            length_output,
        }
    }

    fn update(&mut self) {
        let stage = self.gstate.borrow().stage.clone();
        if let Stage::SelectedRod(i, j) = stage {
            let gstate_borrow = self.gstate.borrow();

            let rod = gstate_borrow.truss.rod(i, j).unwrap();
            self.elastic_inp.set_value(&rod.elastic.to_string());
            self.area_inp.set_value(&rod.area.to_string());

            let rod_info = self.gstate.borrow().truss.rod_info(i, j).unwrap();
            self.length_output.set_value(&rod_info.length.to_string());
            self.stiffness_output
                .set_value(&rod_info.stiffness.to_string());
        }
    }
}

struct PropertyEditor {
    inner: fg::Group,
    rod: RodPropertyEditor,
    node: NodePropertyEditor,
    gstate: Rc<RefCell<State>>,
}

impl PropertyEditor {
    pub fn new(w: i32, h: i32, gstate: Rc<RefCell<State>>) -> Self {
        let mut inner = fg::Group::default().with_size(w, h);
        let rod = RodPropertyEditor::new(w, h, gstate.clone());
        let node = NodePropertyEditor::new(w, h, gstate.clone());
        inner.end();

        inner.draw({
            let gstate = gstate.clone();
            let mut rod = rod.clone();
            let mut node = node.clone();

            move |this| {
                let stage = gstate.borrow().stage.clone();
                match stage {
                    Stage::SelectedRod(..) => {
                        fd::draw_rect_fill(
                            this.x(),
                            this.y(),
                            this.w(),
                            this.h(),
                            fe::Color::Light2,
                        );
                        rod.inner.show();
                        this.draw_child(&mut rod.inner);
                    }
                    Stage::SelectedNode(..) => {
                        fd::draw_rect_fill(
                            this.x(),
                            this.y(),
                            this.w(),
                            this.h(),
                            fe::Color::Light2,
                        );
                        node.inner.show();
                        this.draw_child(&mut node.inner);
                    }
                    _ => {
                        rod.inner.hide();
                        node.inner.hide();
                    }
                }
            }
        });

        Self {
            inner,
            node,
            rod,
            gstate,
        }
    }
    fn update(&mut self) {
        let stage = self.gstate.borrow().stage.clone();
        match stage {
            Stage::SelectedRod(..) => {
                self.rod.update();
            }
            Stage::SelectedNode(..) => {
                self.node.update();
            }
            _ => {}
        }
    }
}

struct StatusBar {
    inner: fg::Group,
}

impl StatusBar {
    pub fn new(w: i32, h: i32, gstate: Rc<RefCell<State>>) -> Self {
        let mut inner = fg::Group::default().with_size(w, h);

        let mut pos_display = fo::Output::default().with_size(200, h).with_pos(0, 0);
        let mut zoom_display = fo::Output::default()
            .with_size(100, h)
            .right_of(&pos_display, 5);

        let mut rod_choice = fm::Choice::default()
            .with_size(200, h)
            .right_of(&zoom_display, 5);
        rod_choice.set_callback({
            let gstate = gstate.clone();
            move |this| {
                let rod = gstate.borrow().rods[this.value() as usize].clone();
                gstate.borrow_mut().rod = rod;
                gstate.borrow_mut().mark_changed();
            }
        });

        let mut show_number = fb::Button::default()
            .with_size(120, h)
            .right_of(&rod_choice, 5);
        show_number.set_callback({
            let gstate = gstate.clone();
            move |_| {
                let show_number = gstate.borrow().show_numbers;
                gstate.borrow_mut().show_numbers = !show_number;
                gstate.borrow_mut().mark_changed();
            }
        });

        let mut show_du = fb::Button::default()
            .with_size(100, h)
            .right_of(&show_number, 5);
        show_du.set_callback({
            let gstate = gstate.clone();
            move |_| {
                let show_du = gstate.borrow().show_du;
                gstate.borrow_mut().show_du = !show_du;
                gstate.borrow_mut().mark_changed();
            }
        });

        let mut auto_calc = fb::Button::default()
            .with_size(120, h)
            .right_of(&show_du, 5);
        auto_calc.set_callback({
            let gstate = gstate.clone();
            move |_| {
                let auto_calc = gstate.borrow().auto_calc;
                gstate.borrow_mut().auto_calc = !auto_calc;
                gstate.borrow_mut().mark_changed();
            }
        });

        let mut show_grid = fb::Button::default()
            .with_size(80, h)
            .right_of(&auto_calc, 5);
        show_grid.set_callback({
            let gstate = gstate.clone();
            move |_| {
                let show_grid = gstate.borrow().show_grid;
                gstate.borrow_mut().show_grid = !show_grid;
                gstate.borrow_mut().mark_changed();
            }
        });

        let mut msg_display = fo::Output::default()
            .with_size(
                w - pos_display.width()
                    - zoom_display.width()
                    - show_number.width()
                    - show_du.width()
                    - auto_calc.width()
                    - 30,
                h,
            )
            .right_of(&show_grid, 5);

        inner.end();

        inner.draw(move |this| {
            let mouse_pos = gstate.borrow().mouse_pos;
            pos_display.set_value(&format!("X = {:.3} Y = {:.3}", mouse_pos.x, mouse_pos.y));

            let zoom = gstate.borrow().zoom * 100.0;
            zoom_display.set_value(&format!("{:.0}%", zoom));

            rod_choice.clear();
            gstate.borrow().rods.iter().for_each(|rod| {
                rod_choice.add_choice(&format!("A = {:.2}, E = {:.2}", rod.area, rod.elastic));
            });
            let rod = gstate.borrow().rod.clone();
            let idx = gstate
                .borrow()
                .rods
                .iter()
                .position(|r| *r == rod)
                .unwrap_or_else(|| 0);
            rod_choice.set_value(idx as i32);

            show_number.set_label(if gstate.borrow().show_numbers {
                "Numbers Shown"
            } else {
                "Numbers Hidden"
            });

            show_du.set_label(if gstate.borrow().show_du {
                "Du Shown"
            } else {
                "Du Hidden"
            });

            auto_calc.set_label(if gstate.borrow().auto_calc {
                "Auto Calc. On"
            } else {
                "Auto Calc. Off"
            });

            show_grid.set_label(if gstate.borrow().show_grid {
                "Grid On"
            } else {
                "Grid Off"
            });

            msg_display.set_value(&gstate.borrow().msg.to_string());

            this.draw_children();
        });

        Self { inner }
    }
}

pub struct Ui {
    inner: fg::Group,
    prop: PropertyEditor,
}

impl Ui {
    pub fn new(w: i32, h: i32, gstate: Rc<RefCell<State>>) -> Self {
        let inner = fg::Group::default_fill();

        const PROP_WIDTH: i32 = 300;
        const BAR_HEIGHT: i32 = 40;

        let canvas_width = w - PROP_WIDTH;
        let canvas_height = h - BAR_HEIGHT;
        let mut canvas = Canvas::new(canvas_width + PROP_WIDTH, canvas_height, gstate.clone());
        canvas.inner.set_pos(0, 0);

        let mut prop = PropertyEditor::new(PROP_WIDTH, canvas_height, gstate.clone());
        prop.inner.set_pos(canvas_width, 0);

        let mut status = StatusBar::new(w, BAR_HEIGHT, gstate.clone());
        status.inner.set_pos(0, canvas_height);

        inner.end();

        Self { inner, prop }
    }

    pub fn redraw(&mut self) {
        self.inner.redraw();
    }

    pub fn update(&mut self) {
        self.prop.update();
    }
}
