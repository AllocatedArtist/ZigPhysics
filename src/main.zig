const std = @import("std");
const rl = @import("raylib");
const shape = @import("shapes/shapes.zig");

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

    var points = [3]rl.Vector3{ rl.Vector3.init(-1.0, 0.0, -1.0), rl.Vector3.init(1.0, 0.0, -1.0), rl.Vector3.init(0.0, 1.0, -1.0) };
    const ground_plane = shape.Plane{ .normal = rl.Vector3.init(0, 1, 0), .distance = 0 };
    const sphere = shape.Sphere.init(rl.Vector3.init(-2, 0, 1), 1.0);
    const box = shape.AABB.init(rl.Vector3.init(2, 0, 3), rl.Vector3.init(0.5, 0.5, 0.5));
    const box_static = shape.AABB.init(rl.Vector3.init(-1.5, 2, -4), rl.Vector3.init(3.0, 2.0, 0.5));

    var box_model = rl.loadModelFromMesh(rl.genMeshCube(1, 1, 1));

    const y_rot = rl.Matrix.rotateY(std.math.degreesToRadians(0.5));
    const x_rot = rl.Matrix.rotateX(std.math.degreesToRadians(0.3));

    const box_rotation = y_rot.multiply(x_rot);

    rl.setTargetFPS(60);

    var hit = rl.RayCollision{ .hit = false, .distance = 0.0, .point = rl.Vector3.zero(), .normal = rl.Vector3.zero() };

    camera.target = camera.position.add(rl.Vector3.init(0, 0, -1));

    while (!rl.windowShouldClose()) {
        rl.beginDrawing();
        rl.clearBackground(rl.Color.black);

        for (&points) |*point| {
            point.* = point.rotateByAxisAngle(rl.Vector3.init(1, 1, 0), rl.getFrameTime());
        }

        const triangle = shape.Triangle.init(points[0], points[1], points[2]);

        if (rl.isMouseButtonDown(.mouse_button_left)) {
            const camera_ray = rl.getScreenToWorldRay(rl.getMousePosition(), camera);
            hit = triangle.rayIntersectTriangleBT(camera_ray);
            if (!hit.hit) {
                hit = box.rayIntersectOBB(box_model.transform, camera_ray);
            }
            if (!hit.hit) {
                hit = box_static.rayIntersectAABB(camera_ray);
            }
            if (!hit.hit) {
                hit = sphere.rayIntersectSphere(camera_ray);
            }
            if (!hit.hit) {
                hit = ground_plane.rayIntersectPlane(camera_ray);
            }
        } else {
            hit.hit = false;
        }

        updateCamera(&camera);

        camera.begin();

        rl.drawGrid(10, 1.0);

        box_model.transform = box_model.transform.multiply(box_rotation);
        rl.drawModel(box_model, box.center, 1.0, rl.Color.orange);
        rl.drawCubeV(box_static.center, box_static.getSize(), rl.Color.pink);

        rl.gl.rlDisableBackfaceCulling();
        rl.drawTriangle3D(points[0], points[1], points[2], rl.Color.red);
        rl.drawSphere(sphere.getCenter(), sphere.getRadius(), rl.Color.maroon);

        if (hit.hit) {
            rl.drawSphere(hit.point, 0.1, rl.Color.yellow);
            rl.drawLine3D(hit.point, hit.point.add(hit.normal), rl.Color.green);
        }

        camera.end();

        rl.endDrawing();
    }
}
