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

fn drawColliders(world: *physics.World, cube_model: rl.Model) void {
    for (world.colliders.items) |collider| {
        switch (collider.shape) {
            .Sphere => |sphere| {
                rl.drawSphere(sphere.getCenter(), sphere.getRadius(), rl.Color.pink);
            },
            .AABB => |box| {
                rl.drawCubeV(box.getCenter(), box.getSize(), rl.Color.red);
            },
            .OBB => |box| {
                var axis: rl.Vector3 = undefined;
                var angle: f32 = undefined;

                rl.Quaternion.fromMatrix(box.orientation.*).toAxisAngle(&axis, &angle);
                rl.drawModelEx(cube_model, box.aabb.getCenter(), axis, std.math.radiansToDegrees(angle), box.aabb.getSize(), rl.Color.dark_blue);
            },
            .Triangle => |triangle| {
                const points = triangle.getPoints();
                rl.drawTriangle3D(points[0], points[1], points[2], rl.Color.sky_blue);
            },
            else => {},
        }
    }
}

pub fn main() !void {
    rl.initWindow(1600, 1480, "Physics");

    rl.setTargetFPS(60);

    camera.target = camera.position.add(rl.Vector3.init(0, 0, -1));

    const allocator = std.heap.page_allocator;

    var sphere_shape = shape.Sphere.init(rl.Vector3.init(0, 0, 0), 1.0);
    var aabb_shape = shape.AABB.init(rl.Vector3.init(-2.0, 0.5, 3.0), rl.Vector3.init(0.5, 0.5, 0.5));

    const initial_points = [3]rl.Vector3{ rl.Vector3.init(0.0, 0.0, 0.0), rl.Vector3.init(4.0, 0.0, 0.0), rl.Vector3.init(2.0, 3.0, 0.0) };

    var triangle_shape = shape.Triangle.init(initial_points[0], initial_points[1], initial_points[2]);
    var rotating_aabb = shape.AABB.init(rl.Vector3.init(-2.0, 2.0, -2.0), rl.Vector3.init(0.5, 0.1, 0.5));

    var orientation = rl.Matrix.rotateX(std.math.degreesToRadians(45.0));

    var world = physics.World.init(allocator);
    defer world.deinit();

    var sphere_collider = physics.Collider.init(physics.ColliderType.createSphere(&sphere_shape));
    var static_box_collider = physics.Collider.init(physics.ColliderType.createAABB(&aabb_shape));
    var triangle_collider = physics.Collider.init(physics.ColliderType.createTriangle(&triangle_shape));
    var rotating_box_collider = physics.Collider.init(physics.ColliderType.createOBB(&rotating_aabb, &orientation));

    rotating_box_collider.layer.setValue(0, true);
    sphere_collider.layer.setValue(1, true);
    triangle_collider.layer.setValue(2, true);
    static_box_collider.layer.setValue(3, true);

    try world.addCollider(&sphere_collider);
    try world.addCollider(&static_box_collider);
    try world.addCollider(&triangle_collider);
    try world.addCollider(&rotating_box_collider);

    const cube_model = rl.loadModelFromMesh(rl.genMeshCube(1.0, 1.0, 1.0));
    defer rl.unloadModel(cube_model);

    var triangle_rot = rl.Matrix.identity();

    while (!rl.windowShouldClose()) {
        rl.beginDrawing();
        rl.clearBackground(rl.Color.black);
        updateCamera(&camera);

        rl.gl.rlDisableBackfaceCulling();

        const camera_ray = rl.getScreenToWorldRay(rl.getMousePosition(), camera);

        const ray_mask = physics.bitmask.initFull();

        const hit = world.getRayIntersection(camera_ray, ray_mask);

        const x_rot = std.math.degreesToRadians(0.05);
        const y_rot = std.math.degreesToRadians(0.1);
        const z_rot = std.math.degreesToRadians(0.15);

        orientation = orientation.multiply(rl.Matrix.rotateXYZ(rl.Vector3.init(x_rot, y_rot, z_rot)));

        var tri_points: [3]rl.Vector3 = undefined;

        const tri_angle = std.math.degreesToRadians(0.5);

        var triangle_transform = rl.Matrix.identity();

        const triangle_scale = rl.Matrix.scale(1.0, 1.0, 1.0);
        const triangle_pos = rl.Matrix.translate(2, 0, 0);
        triangle_rot = triangle_rot.multiply(rl.Matrix.rotateX(tri_angle).multiply(rl.Matrix.rotateY(tri_angle)));

        triangle_transform = triangle_transform.multiply(triangle_scale.multiply(triangle_rot).multiply(triangle_pos));

        tri_points[0] = initial_points[0].transform(triangle_transform);
        tri_points[1] = initial_points[1].transform(triangle_transform);
        tri_points[2] = initial_points[2].transform(triangle_transform);

        triangle_shape = shape.Triangle.init(tri_points[0], tri_points[1], tri_points[2]);

        camera.begin();

        rl.drawGrid(10, 1.0);
        drawColliders(&world, cube_model);

        if (hit.hit) {
            rl.drawSphere(hit.point, 0.05, rl.Color.yellow);
            rl.drawLine3D(hit.point, hit.point.add(hit.normal), rl.Color.green);
        }

        camera.end();

        rl.endDrawing();
    }
}
