#![allow(dead_code)]

pub enum Solution<T> {
    Many,
    None,
    Single(Matrix<T>),
}

pub mod traits {
    use super::{Matrix, Solution};

    use num_traits::{NumAssign, One, Zero};
    use std::ops::{Add, AddAssign, Mul, MulAssign, Sub, SubAssign};

    pub trait MatrixTable<T>: std::ops::Index<(usize, usize), Output = T> {
        fn rows(&self) -> usize;
        fn cols(&self) -> usize;
        fn get(&self, i: usize, j: usize) -> Option<&T>;
        fn clone_data(&self) -> Matrix<T>
        where
            T: Clone,
        {
            let mut data = Vec::with_capacity(self.rows() * self.cols());
            for i in 0..self.rows() {
                for j in 0..self.cols() {
                    data.push(self[(i, j)].clone());
                }
            }
            Matrix {
                data,
                cols: self.cols(),
                rows: self.rows(),
            }
        }
    }

    pub trait MatrixTableMut<T>:
        MatrixTable<T>
        + std::ops::Index<(usize, usize), Output = T>
        + std::ops::IndexMut<(usize, usize)>
    where
        T: Clone,
    {
        fn get_mut(&mut self, i: usize, j: usize) -> Option<&mut T>;
        fn swap(&mut self, i1: usize, j1: usize, i2: usize, j2: usize) {
            (self[(i1, j1)], self[(i2, j2)]) = (self[(i2, j2)].clone(), self[(i1, j1)].clone())
        }
        fn swap_rows(&mut self, i1: usize, i2: usize) {
            for j in 0..self.cols() {
                self.swap(i1, j, i2, j);
            }
        }
        fn swap_cols(&mut self, j1: usize, j2: usize) {
            for i in 0..self.rows() {
                self.swap(i, j1, i, j2);
            }
        }
    }

    pub trait MatrixZero<T>: MatrixTableMut<T>
    where
        T: Zero + Clone,
    {
        fn fill_zeros(&mut self) {
            (0..self.rows()).for_each(|i| (0..self.cols()).for_each(|j| self[(i, j)] = T::zero()));
        }
    }

    pub trait MatrixOne<T>: MatrixTableMut<T>
    where
        T: One + Clone,
    {
        fn fill_ones(&mut self) {
            (0..self.rows()).for_each(|i| (0..self.cols()).for_each(|j| self[(i, j)] = T::one()));
        }
    }

    pub trait Ring:
        One
        + Zero
        + AddAssign<Self>
        + Add<Self, Output = Self>
        + MulAssign<Self>
        + Mul<Self, Output = Self>
        + SubAssign<Self>
        + Sub<Self, Output = Self>
    {
    }

    pub trait Field: Ring + NumAssign {}

    impl<T> Ring for T where T: NumAssign {}
    impl<T> Field for T where T: NumAssign {}

    pub trait MatrixRing<T>: MatrixZero<T>
    where
        T: Ring + Clone,
    {
        fn add_assign<M>(&mut self, rhs: &M) -> Result<(), ()>
        where
            M: MatrixTable<T>,
        {
            if self.rows() != rhs.rows() || self.cols() != rhs.cols() {
                return Err(());
            }

            for i in 0..self.rows() {
                for j in 0..self.cols() {
                    self[(i, j)] += rhs[(i, j)].clone()
                }
            }

            return Ok(());
        }
        fn add<M>(&self, rhs: &M) -> Result<Matrix<T>, ()>
        where
            M: MatrixTable<T>,
        {
            if self.rows() != rhs.rows() || self.cols() != rhs.cols() {
                return Err(());
            }

            let mut r = Matrix::zeros(self.rows(), self.cols());
            for i in 0..self.rows() {
                for j in 0..self.cols() {
                    r[(i, j)] = self[(i, j)].clone() + rhs[(i, j)].clone();
                }
            }

            return Ok(r);
        }
        fn sub_assign<M>(&mut self, rhs: &M) -> Result<(), ()>
        where
            M: MatrixTable<T>,
        {
            if self.rows() != rhs.rows() || self.cols() != rhs.cols() {
                return Err(());
            }

            for i in 0..self.rows() {
                for j in 0..self.cols() {
                    self[(i, j)] -= rhs[(i, j)].clone()
                }
            }

            return Ok(());
        }
        fn sub<M>(&self, rhs: &M) -> Result<Matrix<T>, ()>
        where
            M: MatrixTable<T>,
        {
            if self.rows() != rhs.rows() || self.cols() != rhs.cols() {
                return Err(());
            }

            let mut r = Matrix::zeros(self.rows(), self.cols());
            for i in 0..self.rows() {
                for j in 0..self.cols() {
                    r[(i, j)] = self[(i, j)].clone() - rhs[(i, j)].clone();
                }
            }

            return Ok(r);
        }
        fn dot<M>(&self, rhs: &M) -> Result<Matrix<T>, ()>
        where
            M: MatrixTable<T>,
        {
            if self.cols() != rhs.rows() {
                return Err(());
            }

            let mut r = Matrix::zeros(self.rows(), rhs.cols());
            for i in 0..self.rows() {
                for j in 0..rhs.cols() {
                    r[(i, j)] = (0..self.cols())
                        .map(|k| self[(i, k)].clone() * rhs[(k, j)].clone())
                        .reduce(|accu, x| accu + x)
                        .unwrap_or_else(|| T::zero());
                }
            }
            Ok(r)
        }
        /// Add row `i2` scaled by `k` to row `i1`. `i1` can equal `i2`.
        fn row_add_row(&mut self, i1: usize, k: T, i2: usize) {
            for j in 0..self.cols() {
                let rhs = self[(i2, j)].clone() * k.clone();
                self[(i1, j)] += rhs;
            }
        }
        /// Add column `j2` scaled by `k` to column `j1`. `i1` can equal `i2`.
        fn col_add_col(&mut self, j1: usize, k: T, j2: usize) {
            for i in 0..self.rows() {
                let rhs = self[(i, j2)].clone() * k.clone();
                self[(i, j1)] += rhs;
            }
        }
        fn scaled(mut self, k: T) -> Self
        where
            Self: Sized,
        {
            self.scale(k);
            self
        }
        fn scale(&mut self, k: T)
        where
            Self: Sized,
        {
            for i in 0..self.rows() {
                for j in 0..self.cols() {
                    self[(i, j)] *= k.clone()
                }
            }
        }
    }

    pub trait MatrixField<T>: MatrixRing<T>
    where
        T: Field + Clone + std::fmt::Display,
    {
        fn solve<M: MatrixField<T>>(mut self, mut b: M) -> Option<Solution<T>>
        where
            Self: Sized,
        {
            if self.rows() != b.rows() || b.cols() != 1 {
                return None;
            }

            let mut pivot_col = 0;
            let mut pivot_row = 0;
            let mut pivot_cols = Vec::new();

            while pivot_col < self.cols() && pivot_row < self.rows() {
                if self[(pivot_row, pivot_col)].is_zero() {
                    let mut found_pivot = false;
                    for i in pivot_row + 1..self.rows() {
                        if !self[(i, pivot_col)].is_zero() {
                            self.swap_rows(pivot_row, i);
                            b.swap_rows(pivot_row, i);
                            found_pivot = true;
                            break;
                        }
                    }
                    if !found_pivot {
                        pivot_col += 1;
                        continue;
                    }
                }

                for i in pivot_row + 1..self.rows() {
                    if !self[(i, pivot_col)].is_zero() {
                        let k = self[(i, pivot_col)].clone() / self[(pivot_row, pivot_col)].clone();
                        let k = T::zero() - k;
                        self.row_add_row(i, k.clone(), pivot_col);
                        b.row_add_row(i, k.clone(), pivot_row);
                    }
                }

                pivot_cols.push(pivot_col);
                pivot_col += 1;
                pivot_row += 1;
            }

            if pivot_row < self.rows() {
                for i in pivot_row..self.rows() {
                    if !b[(i, 0)].is_zero() {
                        return Some(Solution::None);
                    }
                }
            }

            if pivot_cols.len() < self.cols() {
                return Some(Solution::Many);
            }

            let mut ans: Matrix<T> = Matrix::zeros(self.cols(), 1);

            for j in (0..self.cols()).rev() {
                let sum = (j + 1..self.cols())
                    .map(|k| self[(j, k)].clone() * ans[(k, 0)].clone())
                    .reduce(|sum, item| sum + item)
                    .unwrap_or(T::zero());
                ans[(j, 0)] = (b[(j, 0)].clone() - sum) / self[(j, j)].clone();
            }

            Some(Solution::Single(ans))
        }
    }

    pub fn fmt_matrix<M, T>(m: &M, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result
    where
        M: MatrixTable<T>,
        T: std::fmt::Display,
    {
        let mut builder = tabled::builder::Builder::default();
        for i in 0..m.rows() {
            let row = (0..m.cols()).map(|j| m[(i, j)].to_string());
            builder.push_record(row);
        }

        use tabled::settings::{Alignment, Style};
        let table = builder
            .build()
            .with(Style::blank())
            .with(Alignment::left())
            .to_string();
        f.write_str(&table)
    }
}
use num_traits::{One, Zero};
use traits::*;

pub struct Matrix<T> {
    data: Vec<T>,
    cols: usize,
    rows: usize,
}

impl<T> Clone for Matrix<T>
where
    T: Clone,
{
    fn clone(&self) -> Self {
        Self {
            data: self.data.clone(),
            cols: self.cols,
            rows: self.rows,
        }
    }
}

impl<T> Matrix<T>
where
    T: Clone + One,
{
    pub fn ones(rows: usize, cols: usize) -> Self {
        Self {
            data: vec![T::one(); rows * cols],
            rows,
            cols,
        }
    }
}

impl<T> Matrix<T>
where
    T: Clone + Zero,
{
    pub fn zeros(rows: usize, cols: usize) -> Self {
        Self {
            data: vec![T::zero(); rows * cols],
            rows,
            cols,
        }
    }
}

impl<T> Matrix<T>
where
    T: Clone + Zero + One,
{
    pub fn diag(rows: usize) -> Self {
        let mut m = Self::zeros(rows, rows);
        for i in 0..rows {
            m[(i, i)] = T::one();
        }
        m
    }
}

impl<T> Matrix<T>
where
    T: Clone,
{
    pub fn new<const R: usize, const C: usize>(data: [T; R * C]) -> Self {
        Self {
            data: data.to_vec(),
            cols: C,
            rows: R,
        }
    }
}

impl<T> Matrix<T> {
    fn index_for(&self, i: usize, j: usize) -> usize {
        i * self.cols + j
    }
}

impl<T> MatrixTable<T> for Matrix<T> {
    fn rows(&self) -> usize {
        self.rows
    }
    fn cols(&self) -> usize {
        self.cols
    }
    fn get(&self, i: usize, j: usize) -> Option<&T> {
        self.data.get(self.index_for(i, j))
    }
}

impl<T> Matrix<T> {
    pub fn slice(&self, i: usize, j: usize, rows: usize, cols: usize) -> Option<Slice<T>> {
        if i + rows > self.rows || j + cols > self.cols {
            return None;
        }
        Some(Slice {
            m: self,
            i,
            j,
            rows,
            cols,
        })
    }
    pub fn slice_mut(
        &mut self,
        i: usize,
        j: usize,
        rows: usize,
        cols: usize,
    ) -> Option<SliceMut<T>> {
        if i + rows > self.rows || j + cols > self.cols {
            return None;
        }
        Some(SliceMut {
            m: self,
            i,
            j,
            rows,
            cols,
        })
    }
    pub fn slice_row(&self, i: usize) -> Option<Slice<T>> {
        self.slice(i, 0, 1, self.cols)
    }
    pub fn slice_col(&self, j: usize) -> Option<Slice<T>> {
        self.slice(0, j, self.rows, 1)
    }
    pub fn slice_row_mut(&mut self, i: usize) -> Option<SliceMut<T>> {
        self.slice_mut(i, 0, 1, self.cols)
    }
    pub fn slice_col_mut(&mut self, j: usize) -> Option<SliceMut<T>> {
        self.slice_mut(0, j, self.rows, 1)
    }
}

impl<T> MatrixTableMut<T> for Matrix<T>
where
    T: Clone,
{
    fn get_mut(&mut self, i: usize, j: usize) -> Option<&mut T> {
        let idx = self.index_for(i, j);
        self.data.get_mut(idx)
    }
    fn swap_rows(&mut self, i1: usize, i2: usize) {
        for j in 0..self.cols {
            let idx1 = self.index_for(i1, j);
            let idx2 = self.index_for(i2, j);
            self.data.swap(idx1, idx2);
        }
    }
    fn swap_cols(&mut self, j1: usize, j2: usize) {
        for i in 0..self.rows {
            let idx1 = self.index_for(i, j1);
            let idx2 = self.index_for(i, j2);
            self.data.swap(idx1, idx2);
        }
    }
}

impl<T> std::ops::Index<(usize, usize)> for Matrix<T> {
    type Output = T;
    fn index(&self, (i, j): (usize, usize)) -> &Self::Output {
        &self.data[self.index_for(i, j)]
    }
}

impl<T> std::ops::IndexMut<(usize, usize)> for Matrix<T> {
    fn index_mut(&mut self, (i, j): (usize, usize)) -> &mut Self::Output {
        let idx = self.index_for(i, j);
        &mut self.data[idx]
    }
}

impl<T> std::fmt::Display for Matrix<T>
where
    T: std::fmt::Display,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        fmt_matrix(self, f)
    }
}

impl<T> MatrixOne<T> for Matrix<T> where T: One + Clone {}
impl<T> MatrixZero<T> for Matrix<T> where T: Zero + Clone {}
impl<T> MatrixRing<T> for Matrix<T> where T: Ring + Clone {}
impl<T> MatrixField<T> for Matrix<T> where T: Field + Clone + std::fmt::Display {}

pub struct Slice<'m, T> {
    m: &'m Matrix<T>,
    i: usize,
    j: usize,
    rows: usize,
    cols: usize,
}

impl<'m, T> std::ops::Index<(usize, usize)> for Slice<'m, T> {
    type Output = T;
    fn index(&self, (i, j): (usize, usize)) -> &Self::Output {
        &self.m[(self.i + i, self.j + j)]
    }
}

impl<'m, T> MatrixTable<T> for Slice<'m, T> {
    fn rows(&self) -> usize {
        self.rows
    }
    fn cols(&self) -> usize {
        self.cols
    }
    fn get(&self, i: usize, j: usize) -> Option<&T> {
        self.m.get(self.i + i, self.j + j)
    }
}

impl<'m, T> std::fmt::Display for Slice<'m, T>
where
    T: std::fmt::Display,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        fmt_matrix(self, f)
    }
}

pub struct SliceMut<'m, T> {
    m: &'m mut Matrix<T>,
    i: usize,
    j: usize,
    rows: usize,
    cols: usize,
}

impl<'m, T> std::ops::Index<(usize, usize)> for SliceMut<'m, T> {
    type Output = T;
    fn index(&self, (i, j): (usize, usize)) -> &Self::Output {
        &self.m[(self.i + i, self.j + j)]
    }
}

impl<'m, T> std::ops::IndexMut<(usize, usize)> for SliceMut<'m, T> {
    fn index_mut(&mut self, (i, j): (usize, usize)) -> &mut Self::Output {
        &mut self.m[(self.i + i, self.j + j)]
    }
}

impl<'m, T> MatrixTable<T> for SliceMut<'m, T> {
    fn rows(&self) -> usize {
        self.rows
    }
    fn cols(&self) -> usize {
        self.cols
    }
    fn get(&self, i: usize, j: usize) -> Option<&T> {
        self.m.get(self.i + i, self.j + j)
    }
}

impl<'m, T> MatrixTableMut<T> for SliceMut<'m, T>
where
    T: Clone,
{
    fn get_mut(&mut self, i: usize, j: usize) -> Option<&mut T> {
        self.m.get_mut(self.i + i, self.j + j)
    }
}

impl<'m, T> std::fmt::Display for SliceMut<'m, T>
where
    T: std::fmt::Display,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        fmt_matrix(self, f)
    }
}

impl<'m, T> MatrixOne<T> for SliceMut<'m, T> where T: One + Clone {}
impl<'m, T> MatrixZero<T> for SliceMut<'m, T> where T: Zero + Clone {}
impl<'m, T> MatrixRing<T> for SliceMut<'m, T> where T: Ring + Clone {}
impl<'m, T> MatrixField<T> for SliceMut<'m, T> where T: Field + Clone + std::fmt::Display {}
