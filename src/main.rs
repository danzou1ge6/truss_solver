use fltk::prelude::*;
use std::cell::RefCell;
use std::rc::Rc;
use truss_solver::*;

const WIDTH: i32 = 1600;
const HEIGHT: i32 = 900;

fn main() {
    let app = fltk::app::App::default().with_scheme(fltk::app::Scheme::Base);
    let mut my_window = fltk::window::Window::default().with_size(WIDTH, HEIGHT).with_label("桁架力学计算器");

    let gstate = Rc::new(RefCell::new(State::default()));
    let mut ui = Ui::new(WIDTH, HEIGHT, gstate.clone());

    my_window.end();
    my_window.show();

    while app.wait() {
        if gstate.borrow_mut().consume_change_flag() {
            ui.update();
            ui.redraw();
        }
    }
}
