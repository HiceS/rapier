use rapier3d::prelude::*;
use rapier_testbed3d::Testbed;

fn create_revolute_joints(
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    impulse_joints: &mut ImpulseJointSet,
    multibody_joints: &mut MultibodyJointSet,
    origin: Point<f32>,
    num: usize,
    use_articulations: bool,
) {
    let rad = 0.4;
    let shift = 2.0;

    let ground = RigidBodyBuilder::fixed().translation(vector![origin.x, origin.y, 0.0]);
    let curr_parent = bodies.insert(ground);
    let collider = ColliderBuilder::cuboid(rad, rad, rad);
    colliders.insert_with_parent(collider, curr_parent, bodies);

    for i in 0..num {
        // Create four bodies.
        let z = origin.z + i as f32 * shift * 2.0 + shift;

        let positions = [
            Isometry::translation(origin.x, origin.y, z),
            Isometry::translation(origin.x + shift, origin.y, origin.z), // this will be auto corrected
        ];

        let mut handles = [curr_parent; 2];
        for k in 0..2 {
            let rigid_body = RigidBodyBuilder::dynamic().position(positions[k]);
            handles[k] = bodies.insert(rigid_body);
            let collider = ColliderBuilder::cuboid(rad, rad, rad);
            colliders.insert_with_parent(collider, handles[k], bodies);
        }

        // Setup four impulse_joints.
        let x = Vector::x_axis();
        let z = Vector::z_axis();
        let revs = [
            RevoluteJointBuilder::new(z)
                .local_anchor2(point![0.0, 0.0, -shift])
                .motor_velocity(-2.0, 1000.0),
            RevoluteJointBuilder::new(x)
                .local_anchor2(point![-shift, 0.0, 0.0])
                .motor_velocity(-4.0, 1000.0),
        ];

        if use_articulations {
            multibody_joints.insert(curr_parent, handles[0], revs[0], true);
            multibody_joints.insert(handles[0], handles[1], revs[1], true);
        } else {
            let revs = [
                impulse_joints.insert(curr_parent, handles[0], revs[0], true),
                impulse_joints.insert(curr_parent, handles[1], revs[1], true)
            ];

            // will take joint revs[1] and motion link revs[0]
            impulse_joints.get_mut(revs[1]).unwrap().data.set_motion_link(&MotionLink { joint_handle: revs[0], ratio: 0.5, reversed: false });
        }



        // curr_parent = handles[3];
    }
}

pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();

    create_revolute_joints(
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        point![0.0, 0.0, 0.0],
        1,
        false,
    );

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![10.0, 10.0, 10.0], Point::origin());
}
