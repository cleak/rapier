// Based on examples3d/character_controller3.rs

use rapier3d::prelude::*;
use rapier_testbed3d::Testbed;
use std::f32::consts::PI;

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    /*
     * Ground
     */
    let ground_size = 7.5;
    let ground_height = 0.1;

    let rigid_body = RigidBodyBuilder::fixed();
    let floor_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::cuboid(ground_size, ground_height, ground_size)
        .translation(-Vector::y() * ground_height)
        // Small rotation collisions have a non-zero y-component
        .rotation(Vector::z() * PI / 64.0);
    colliders.insert_with_parent(collider, floor_handle, &mut bodies);

    /*
     * Create arena walls.
     */

    // Outer wall half dimensions (length, height, depth)
    let wall_dims = [6.0, 0.3, 0.1];
    for i in 0..4 {
        let rot_rads = PI * (i as f32) / 2.0;

        let collider = ColliderBuilder::cuboid(wall_dims[0], wall_dims[1], wall_dims[2])
            .translation(vector![
                rot_rads.cos() * (wall_dims[0] + wall_dims[2]),
                wall_dims[1],
                rot_rads.sin() * (wall_dims[0] + wall_dims[2])
            ])
            .rotation(Vector::y() * (-rot_rads + PI / 2.0));
        let collider_handle = colliders.insert(collider);
        testbed.set_initial_collider_color(collider_handle, [0.0, 1.0, 0.2]);
    }

    // Inner wall half dimensions (length, height, depth)
    let wall_dims = [2.0, 0.25, 0.1];
    let rot_offset = PI / 8.0;
    for i in 0..3 {
        let rot_rads = PI * (i as f32) / 2.0 + rot_offset;

        let collider = ColliderBuilder::cuboid(wall_dims[0], wall_dims[1], wall_dims[2])
            .translation(vector![
                rot_rads.cos() * (wall_dims[0] + wall_dims[2]),
                wall_dims[1],
                rot_rads.sin() * (wall_dims[0] + wall_dims[2])
            ])
            .rotation(Vector::y() * (-rot_rads + PI / 2.0));
        let collider_handle = colliders.insert(collider);
        testbed.set_initial_collider_color(collider_handle, [0.0, 0.5, 0.5]);
    }

    /*
     * Character we will control manually.
     */
    let rigid_body =
        RigidBodyBuilder::kinematic_position_based().translation(vector![-3.0, 5.0, 0.0]);
    let character_handle = bodies.insert(rigid_body);
    let collider = ColliderBuilder::capsule_y(0.3, 0.15);
    colliders.insert_with_parent(collider, character_handle, &mut bodies);

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.set_character_body(character_handle);
    testbed.look_at(point!(10.0, 10.0, 10.0), Point::origin());
}
