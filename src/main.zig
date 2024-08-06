const std = @import("std");
const rl = @import("raylib");
const shape = @import("shapes/shapes.zig");
const physics = @import("physics/physics.zig");

// zig fmt: off
var camera = rl.Camera3D {
  .position = rl.Vector3.init(0, 1, 3),
  .target = rl.Vector3.init(0, 0, -1),
  .up = rl.Vector3.init(0, 1, 0),
  .fovy = 90,
  .projection = .camera_perspective
};
// zig fmt: on

var yaw: f32 = 0;
var pitch: f32 = 0;
var forward = rl.Vector3.init(0, 0, -1);

fn updateCamera(cam: *rl.Camera) void {
    const right = forward.crossProduct(cam.up);

    var dir = rl.Vector3.zero();
    if (rl.isKeyDown(.key_w)) {
        dir = dir.add(forward);
    }

    if (rl.isKeyDown(.key_s)) {
        dir = dir.add(forward.negate());
    }

    if (rl.isKeyDown(.key_a)) {
        dir = dir.add(right.negate());
    }

    if (rl.isKeyDown(.key_d)) {
        dir = dir.add(right);
    }

    dir = dir.normalize();
    const speed = 5;
    const delta = rl.getFrameTime();

    cam.position = cam.position.add(dir.scale(speed * delta));
    cam.target = cam.position.add(forward);

    if (!rl.isMouseButtonDown(.mouse_button_right)) {
        return;
    }

    const sensitivity = 0.1;
    yaw -= rl.getMouseDelta().x * sensitivity;
    yaw = @mod(yaw, 360);
    pitch -= rl.getMouseDelta().y * sensitivity;
    pitch = std.math.clamp(pitch, -89, 89);

    forward = rl.Vector3.init(0, 0, -1);

    const yaw_rad = std.math.degreesToRadians(yaw);
    const pitch_rad = std.math.degreesToRadians(pitch);

    const rot = rl.Quaternion.fromEuler(pitch_rad, yaw_rad, 0);
    forward = forward.rotateByQuaternion(rot);
}

pub fn main() !void {
    rl.initWindow(1600, 1480, "Physics");

    rl.setTargetFPS(60);

    camera.target = camera.position.add(rl.Vector3.init(0, 0, -1));

    const allocator = std.heap.page_allocator;

    const smiley = rl.loadTexture("assets/smiley-face.png");
    defer rl.unloadTexture(smiley);

    var smiley_ball = rl.loadModelFromMesh(rl.genMeshSphere(1.0, 10.0, 10.0));
    defer rl.unloadModel(smiley_ball);

    rl.setMaterialTexture(@ptrCast(&smiley_ball.materials[0]), .material_map_albedo, smiley);

    const cube_model = rl.loadModelFromMesh(rl.genMeshCube(1.0, 1.0, 1.0));
    defer rl.unloadModel(cube_model);

    var world = physics.World.init(allocator);
    defer world.deinit();

    var box_shape = shape.AABB.init(rl.Vector3.init(0.5, 0.5, 0.5));
    var box_collider = physics.Collider.init(physics.ColliderType.createAABB(&box_shape));
    box_collider.layer.setValue(0, true);

    var big_box_shape = shape.AABB.init(rl.Vector3.init(1.0, 1.0, 1.0));
    var big_box_collider = physics.Collider.init(physics.ColliderType.createOBB(&big_box_shape));
    big_box_collider.layer.setValue(0, true);

    var box_body = physics.RigidBody.init(&box_collider, rl.Vector3.init(0, 10, 0));
    var heavy_box_body = physics.RigidBody.init(&big_box_collider, rl.Vector3.init(0, 10, 2));

    var sphere_shape = shape.Sphere.init(0.5);
    var sphere_collider = physics.Collider.init(physics.ColliderType.createSphere(&sphere_shape));
    sphere_collider.layer.setValue(0, true);

    var sphere_body = physics.RigidBody.init(&sphere_collider, rl.Vector3.init(-3, 5, 0));

    sphere_body.setMass(10);
    sphere_body.damping_factor = 0.56;
    sphere_body.use_gravity = true;

    heavy_box_body.setMass(15);
    heavy_box_body.damping_factor = 0.6;
    heavy_box_body.use_gravity = true;

    box_body.setMass(10);
    box_body.damping_factor = 0.6;
    box_body.use_gravity = true;

    try world.addRigidBody(&box_body);
    try world.addRigidBody(&heavy_box_body);
    try world.addRigidBody(&sphere_body);

    while (!rl.windowShouldClose()) {
        rl.beginDrawing();
        rl.clearBackground(rl.Color.black);
        updateCamera(&camera);

        for (world.rigid_bodies.items) |body| {
            if (body.position.y <= 0) {
                body.addForce(world.gravity.negate().scale(body.mass * 10.0));
            }
        }

        world.integrate(rl.getFrameTime());

        rl.gl.rlDisableBackfaceCulling();

        const camera_ray = rl.getScreenToWorldRay(rl.getMousePosition(), camera);
        const ray_mask = physics.bitmask.initFull();

        const hit = world.getRayIntersection(camera_ray, ray_mask);

        if (rl.isMouseButtonDown(rl.MouseButton.mouse_button_left) and hit.body != null) {
            const info = hit.info;
            var body = hit.body.?;
            const force = camera_ray.direction.scale(100);
            body.addForceAtPosition(force, info.point);
        }

        camera.begin();

        rl.drawGrid(10, 1.0);

        for (world.rigid_bodies.items, 0..) |body, index| {
            if (index == 0) {
                rl.drawCubeV(body.position, body.collider.shape.AABB.getSize(), rl.Color.ray_white);
            } else if (index == 1) {
                var axis: rl.Vector3 = undefined;
                var angle: f32 = undefined;

                rl.Quaternion.fromMatrix(heavy_box_body.orientation).toAxisAngle(&axis, &angle);

                angle = std.math.radiansToDegrees(angle);

                rl.drawModelEx(cube_model, heavy_box_body.position, axis, angle, heavy_box_body.collider.shape.OBB.getSize(), rl.Color.pink);
            } else if (index == 2) {
                var axis: rl.Vector3 = undefined;
                var angle: f32 = undefined;
                rl.Quaternion.fromMatrix(sphere_body.orientation).toAxisAngle(&axis, &angle);

                angle = std.math.radiansToDegrees(angle);

                rl.drawModelEx(smiley_ball, sphere_body.position, axis, angle, rl.Vector3.init(0.5, 0.5, 0.5), rl.Color.white);
            }
        }

        camera.end();

        rl.endDrawing();
    }
}
