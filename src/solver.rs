use std::collections::BTreeMap;

use crate::matrix::traits::*;
use crate::matrix::{Matrix, Solution};

#[derive(Clone, Debug)]
pub enum Constrain {
    Force { phi: f64, f: f64 },
    Hinge,
    Slide(f64),
    None,
}

#[derive(Clone, Debug, Copy)]
pub struct Vec2D {
    pub x: f64,
    pub y: f64,
}

impl Vec2D {
    pub fn new_polar(phi: f64) -> Self {
        Self {
            x: phi.cos(),
            y: phi.sin(),
        }
    }
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
    pub fn distance(&self, rhs: &Self) -> f64 {
        ((self.x - rhs.x).powi(2) + (self.y - rhs.y).powi(2)).sqrt()
    }
    pub fn rotated_quad(&self) -> Self {
        Self {
            x: -self.y,
            y: self.x,
        }
    }
    pub fn length(&self) -> f64 {
        (self.x.powi(2) + self.y.powi(2)).sqrt()
    }
    pub fn cross(&self, rhs: &Self) -> f64 {
        self.x * rhs.y - self.y * rhs.x
    }
    pub fn normalize(&mut self) {
        let n = self.length();
        self.x /= n;
        self.y /= n;
    }
    pub fn normalized(mut self) -> Self {
        self.normalize();
        self
    }
    pub fn counter_clockwise_rotated(self, phi: f64) -> Self {
        Self::new(
            self.x * phi.cos() - self.y * phi.sin(),
            self.x * phi.sin() + self.y * phi.cos(),
        )
    }
    /// `self`'s distance to line determined by two points `a` and `b`
    pub fn distance_to_line(&self, a: Vec2D, b: Vec2D) -> f64 {
        let n = (a - b).rotated_quad().normalized();
        ((*self - a) * n).abs()
    }
}

impl std::ops::Add<Vec2D> for Vec2D {
    type Output = Vec2D;
    fn add(self, rhs: Self) -> Self::Output {
        Vec2D::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl std::ops::AddAssign<Vec2D> for Vec2D {
    fn add_assign(&mut self, rhs: Vec2D) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl std::ops::Sub<Vec2D> for Vec2D {
    type Output = Vec2D;
    fn sub(self, rhs: Vec2D) -> Self::Output {
        Vec2D::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl std::ops::SubAssign<Vec2D> for Vec2D {
    fn sub_assign(&mut self, rhs: Vec2D) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

impl std::ops::Mul<Vec2D> for Vec2D {
    type Output = f64;
    fn mul(self, rhs: Vec2D) -> Self::Output {
        self.x * rhs.x + self.y * rhs.y
    }
}

impl std::ops::Mul<Vec2D> for f64 {
    type Output = Vec2D;
    fn mul(self, rhs: Vec2D) -> Self::Output {
        Vec2D::new(self * rhs.x, self * rhs.y)
    }
}

#[derive(Debug, Clone)]
pub struct Node {
    pub(crate) pos: Vec2D,
    pub(crate) constrain: Constrain,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Rod {
    pub elastic: f64,
    pub area: f64,
}

impl Default for Rod {
    fn default() -> Self {
        Rod {
            elastic: 10000.0,
            area: 2.0,
        }
    }
}

impl std::hash::Hash for Rod {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        state.write_u64(unsafe { std::mem::transmute::<_, _>(self.elastic) });
        state.write_u64(unsafe { std::mem::transmute::<_, _>(self.area) });
    }
}

impl Eq for Rod {}

#[derive(PartialEq, Eq, PartialOrd, Ord, Debug, Clone, Copy)]
pub struct NodeId(usize);

pub struct Truss {
    pub(crate) graph: BTreeMap<NodeId, BTreeMap<NodeId, Rod>>,
    pub(crate) nodes: BTreeMap<NodeId, Node>,
    pub(crate) node_id_cnt: usize,
}

#[derive(Debug, Clone)]
pub enum TrussSolveError {
    Unbound,
    Overbound,
    Empty,
}

impl std::fmt::Display for TrussSolveError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        use TrussSolveError::*;
        match self {
            Overbound => f.write_str("Error: Overbound"),
            Unbound => f.write_str("Error: Unbound"),
            Empty => f.write_str("Error: Emtpy truss"),
        }
    }
}

pub struct RodInfo {
    pub length: f64,
    pub stiffness: f64,
    pub cos: f64,
    pub sin: f64,
}

/// Editting truss
impl Truss {
    pub fn new() -> Self {
        Self {
            graph: BTreeMap::new(),
            nodes: BTreeMap::new(),
            node_id_cnt: 0,
        }
    }
    pub fn add_node(&mut self, pos: Vec2D) -> NodeId {
        let node_id = NodeId(self.node_id_cnt);
        self.node_id_cnt += 1;
        self.graph.insert(node_id, BTreeMap::new());
        self.nodes.insert(
            node_id,
            Node {
                pos,
                constrain: Constrain::None,
            },
        );
        node_id
    }
    /// Remove a node. Returns `Err` if such node doesn't exist
    pub fn remove_node(&mut self, i: NodeId) -> Result<(), ()> {
        if !self.graph.contains_key(&i) {
            return Err(());
        }

        for j in self.graph[&i]
            .keys()
            .copied()
            .collect::<Vec<_>>()
            .into_iter()
        {
            self.graph.get_mut(&j).unwrap().remove(&i);
        }

        self.graph.remove(&i);
        self.nodes.remove(&i);

        Ok(())
    }
    /// Insert a rod. Returns `Err` if undirected edge (`i`, `j`) already exists or one of `i`, `j` doesn't exist.
    pub fn add_rod(&mut self, i: NodeId, j: NodeId, rod: Rod) -> Result<(), ()> {
        if i == j {
            return Err(());
        }

        if !self.graph.contains_key(&j) || !self.graph.contains_key(&i) {
            return Err(());
        }

        if self.graph[&i].contains_key(&j) {
            return Err(());
        }

        self.graph.get_mut(&i).unwrap().insert(j, rod.clone());
        self.graph.get_mut(&j).unwrap().insert(i, rod);

        return Ok(());
    }
    /// Make sure `i` exists!
    fn deg(&self, i: NodeId) -> usize {
        self.graph[&i].len()
    }
    /// Make sure `i` exists!
    fn garbage_collect_node(&mut self, i: NodeId) {
        if self.deg(i) == 0 {
            self.remove_node(i).unwrap();
        }
    }
    /// Remove a rod. Returns `Err` if such rod doesn't exist.
    pub fn remove_rod(&mut self, i: NodeId, j: NodeId) -> Result<Rod, ()> {
        if let Some(ni) = self.graph.get_mut(&i) {
            if let Some(rod) = ni.remove(&j) {
                let nj = self.graph.get_mut(&j).unwrap();
                nj.remove(&i);

                self.garbage_collect_node(i);
                self.garbage_collect_node(j);

                return Ok(rod);
            }
        }
        Err(())
    }
}

/// Querying truss nodes and rods by position
impl Truss {
    /// Returns the first node whose distance to `pos` is <= `r`
    pub fn node_by_pos(&self, pos: &Vec2D, r: f64) -> Option<NodeId> {
        self.nodes
            .iter()
            .find(|(_, node)| node.pos.distance(pos) <= r)
            .map(|(id, _)| *id)
    }
    /// Returns the first rod whose distance to `pos` is <= `r`
    pub fn rod_by_pos(&self, pos: &Vec2D, r: f64) -> Option<(NodeId, NodeId)> {
        fn between(a: f64, x: f64, b: f64, r: f64) -> bool {
            if a <= b {
                a - r <= x && x <= b + r
            } else {
                b - r <= x && x <= a + r
            }
        }
        self.graph
            .iter()
            .map(|(i, ni)| ni.iter().map(|(j, rod)| (*i, *j, rod)))
            .flatten()
            .filter(|(i, j, _rod)| {
                let posi = self.nodes[i].pos;
                let posj = self.nodes[j].pos;
                between(posi.x, pos.x, posj.x, r)
                    && between(posi.y, pos.y, posj.y, r)
                    && pos.distance_to_line(posi, posj) <= r
            })
            .max_by(|(i1, j1, _rod1), (i2, j2, _rod2)| {
                let posi1 = self.nodes[i1].pos;
                let posj1 = self.nodes[j1].pos;
                let posi2 = self.nodes[i2].pos;
                let posj2 = self.nodes[j2].pos;
                pos.distance_to_line(posi1, posj1)
                    .partial_cmp(&pos.distance_to_line(posi2, posj2))
                    .unwrap_or(std::cmp::Ordering::Greater)
            })
            .map(|(i, j, _rod)| (i, j))
    }
}

/// Solving truss
impl Truss {
    /// Get [`Rod`] `i, j`.
    pub fn rod(&self, i: NodeId, j: NodeId) -> Option<&Rod> {
        let ni = self.graph.get(&i)?;
        ni.get(&j)
    }
    /// Get mutable [`Rod`] `i, j`.
    pub fn rod_mut(&mut self, i: NodeId, j: NodeId) -> Option<&mut Rod> {
        let ni = self.graph.get_mut(&i)?;
        ni.get_mut(&j)
    }
    /// Calculate some characteristic of [`Rod`] `i, j`.
    pub fn rod_info(&self, i: NodeId, j: NodeId) -> Option<RodInfo> {
        let length = self.nodes[&i].pos.distance(&self.nodes[&j].pos);

        let rod = self.rod(i, j)?;
        let k = rod.elastic * rod.area / length;

        let cos = (self.nodes[&j].pos.x - self.nodes[&i].pos.x) / length;
        let sin = (self.nodes[&j].pos.y - self.nodes[&i].pos.y) / length;

        Some(RodInfo {
            length,
            stiffness: k,
            cos,
            sin,
        })
    }
    pub fn rods(&self) -> impl Iterator<Item = &Rod> {
        self.graph.values().map(|edge| edge.values()).flatten()
    }
    /// Calculate stiffness matrix for rod.
    pub fn rod_stiffness(&self, i: NodeId, j: NodeId) -> Option<Matrix<f64>> {
        let rinfo = self.rod_info(i, j)?;

        let cos = rinfo.cos;
        let sin = rinfo.sin;
        let cos2 = cos.powi(2);
        let sin2 = sin.powi(2);
        let sincos = sin * cos;

        #[rustfmt::skip]
        let r = Matrix::new::<4, 4>([
             cos2  ,  sincos, -cos2  , -sincos,
             sincos,  sin2  , -sincos, -sin2,
            -cos2  , -sincos,  cos2  ,  sincos,
            -sincos, -sin2  ,  sincos,  sin2
        ]).scaled(rinfo.stiffness);

        Some(r)
    }
    /// Calculate force on rod.
    pub fn rods_force(&self, du: &Displacement) -> Forces {
        let mut forces = BTreeMap::new();

        for (i, edges) in self.graph.iter() {
            for j in edges.keys() {
                let rinfo = self.rod_info(*i, *j).unwrap();
                let ui = du.get(*i).unwrap();
                let uj = du.get(*j).unwrap();
                let f = rinfo.stiffness * (rinfo.cos * (ui.x - uj.x) + rinfo.sin * (ui.y - uj.y));
                forces.insert((*i, *j), f);
            }
        }

        let max_abs_force = forces
            .values()
            .copied()
            .reduce(|f1, f2| f1.abs().max(f2.abs()));
        Forces {
            f: forces,
            abs_max: max_abs_force,
        }
    }
    /// Calculate stiffness matrix for whole truss.
    pub fn truss_stiffness(&self) -> StiffnessMatrix {
        let mut k = StiffnessMatrix::new(self.nodes.keys().copied());

        for (i, adjs) in self.graph.iter() {
            for (j, _) in adjs.iter().filter(|(j, _)| *i < **j) {
                let rod_stiffness = self.rod_stiffness(*i, *j).unwrap();
                k.add_rod_stiffness(*i, *j, rod_stiffness);
            }
        }

        k
    }
    /// Apply constraints to truss stiffness matrix.
    fn apply_constraints(&self, sm: StiffnessMatrix) -> (StiffnessMatrix, Matrix<f64>) {
        let mut forces = Matrix::zeros(sm.m.rows(), 1);

        let mut col_to_node = vec![(NodeId(0), XOrY::X); self.nodes.len() * 2];
        sm.indices
            .iter()
            .for_each(|((node_id, x_or_y), col)| col_to_node[*col] = (*node_id, *x_or_y));
        let mut left_col_mark = vec![true; sm.m.rows()];

        let StiffnessMatrix { mut m, indices } = sm;

        for (node_id, node) in self.nodes.iter() {
            let ax = indices[&(*node_id, XOrY::X)];
            let ay = indices[&(*node_id, XOrY::Y)];

            match node.constrain.clone() {
                Constrain::None => {}
                Constrain::Hinge => {
                    left_col_mark[ax] = false;
                    left_col_mark[ay] = false;
                }
                Constrain::Force { phi, f } => {
                    forces[(ax, 0)] = phi.cos() * f;
                    forces[(ay, 0)] = phi.sin() * f;
                }
                Constrain::Slide(phi) => {
                    // Force at `node_i` must be parallel to `n`
                    let n = Vec2D::new(phi.cos(), phi.sin());
                    let rotated_n = n.rotated_quad();
                    m.slice_row_mut(ax).unwrap().scale(rotated_n.x);
                    m.row_add_row(ax, rotated_n.y, ay);

                    // Displacement at `node_i` must be orthogonal to `n`
                    let mut row = m.slice_row_mut(ay).unwrap();
                    row.fill_zeros();
                    row[(0, ax)] = n.x;
                    row[(0, ay)] = n.y;
                }
            }
        }

        // Filter off zero columns and "don't care" rows

        let new_cols = left_col_mark.iter().filter(|x| **x).count();
        let mut new_m = Matrix::zeros(new_cols, new_cols);

        for (a, a0) in (0..m.rows()).filter(|a| left_col_mark[*a]).enumerate() {
            for (b, b0) in (0..m.cols()).filter(|b| left_col_mark[*b]).enumerate() {
                new_m[(a, b)] = m[(a0, b0)];
            }
        }

        let mut new_forces = Matrix::zeros(new_cols, 1);
        for (a, a0) in (0..m.rows()).filter(|a| left_col_mark[*a]).enumerate() {
            new_forces[(a, 0)] = forces[(a0, 0)];
        }

        let new_col_to_node = col_to_node
            .into_iter()
            .enumerate()
            .filter(|(a, _)| left_col_mark[*a])
            .map(|(_, x)| x)
            .collect::<Vec<_>>();
        let new_indices = new_col_to_node
            .into_iter()
            .enumerate()
            .map(|(a, (node_id, x_or_y))| ((node_id, x_or_y), a))
            .collect();

        (
            StiffnessMatrix {
                m: new_m,
                indices: new_indices,
            },
            new_forces,
        )
    }

    pub fn solve(&self) -> Result<Displacement, TrussSolveError> {
        if self.nodes.is_empty() {
            return Err(TrussSolveError::Empty);
        }
        let sm = self.truss_stiffness();
        let (sm, forces) = self.apply_constraints(sm);

        match sm.m.solve(forces).unwrap() {
            Solution::Single(du) => {
                let mut indices: BTreeMap<_, _> = sm
                    .indices
                    .into_iter()
                    .map(|((node_id, x_or_y), col)| ((node_id, x_or_y), Some(col)))
                    .collect();

                for node_id in self.nodes.keys() {
                    if !indices.contains_key(&(*node_id, XOrY::X)) {
                        indices.insert((*node_id, XOrY::X), None);
                        indices.insert((*node_id, XOrY::Y), None);
                    }
                }

                Ok(Displacement { m: du, indices })
            }
            Solution::Many => return Err(TrussSolveError::Unbound),
            Solution::None => return Err(TrussSolveError::Overbound),
        }
    }
}

#[derive(Clone)]
pub struct Displacement {
    m: Matrix<f64>,
    indices: BTreeMap<(NodeId, XOrY), Option<usize>>,
}

impl Displacement {
    pub fn get_x(&self, node_id: NodeId) -> Option<f64> {
        self.indices
            .get(&(node_id, XOrY::X))
            .map(|a| a.map_or(0.0, |a| self.m[(a, 0)]))
    }
    pub fn get_y(&self, node_id: NodeId) -> Option<f64> {
        self.indices
            .get(&(node_id, XOrY::Y))
            .map(|a| a.map_or(0.0, |a| self.m[(a, 0)]))
    }
    pub fn get(&self, node_id: NodeId) -> Option<Vec2D> {
        self.get_x(node_id)
            .map(|x| Vec2D::new(x, self.get_y(node_id).unwrap()))
    }
}

#[derive(PartialEq, Eq, PartialOrd, Ord, Clone, Copy)]
enum XOrY {
    X,
    Y,
}

pub struct StiffnessMatrix {
    m: Matrix<f64>,
    indices: BTreeMap<(NodeId, XOrY), usize>,
}

impl StiffnessMatrix {
    pub fn new(node_ids: impl Iterator<Item = NodeId>) -> Self {
        let indices: BTreeMap<_, _> = node_ids
            .enumerate()
            .map(|(i, node_id)| {
                [((node_id, XOrY::X), 2 * i), ((node_id, XOrY::Y), 2 * i + 1)].into_iter()
            })
            .flatten()
            .collect();
        let n_nodes = indices.len();
        Self {
            m: Matrix::zeros(n_nodes, n_nodes),
            indices,
        }
    }
    pub fn add_rod_stiffness(&mut self, i: NodeId, j: NodeId, rod_stiffness: Matrix<f64>) {
        let (ix, iy) = (self.indices[&(i, XOrY::X)], self.indices[&(i, XOrY::Y)]);
        let (jx, jy) = (self.indices[&(j, XOrY::X)], self.indices[&(j, XOrY::Y)]);

        for (rs_a, m_a) in [ix, iy, jx, jy].into_iter().enumerate() {
            for (rs_b, m_b) in [ix, iy, jx, jy].into_iter().enumerate() {
                self.m[(m_a, m_b)] += rod_stiffness[(rs_a, rs_b)];
            }
        }
    }
}

pub struct Forces {
    f: BTreeMap<(NodeId, NodeId), f64>,
    abs_max: Option<f64>,
}

impl Forces {
    pub fn get(&self, i: NodeId, j: NodeId) -> Option<f64> {
        self.f.get(&(i, j)).copied()
    }
    pub fn abs_max(&self) -> Option<f64> {
        self.abs_max
    }
}
