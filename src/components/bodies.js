
"use strict";

class World {

	constructor(setup) {
		this.DEBUG = setup.DEBUG || false;
		this.gravity = setup.gravity || 50;
		this.iterations = setup.iterations || 10;
		this.timeStep = setup.timeStep || 1 / 30;
		this.invDT = 1 / this.timeStep;
		this.friction = setup.friction || 0.2;
		this.bodies = [];
		this.joints = [];
		this.contacts = [];
		this.shapes = {};
		this.maxy = setup.maxy || 100000;
		this.ie1x = 0.0;
		this.ie1y = 0.0;
		this.ie2x = 0.0;
		this.ie2y = 0.0;
		this.oe1x = 0.0;
		this.oe1y = 0.0;
		this.oe2x = 0.0;
		this.oe2y = 0.0;
		this.bodyCount = 0;
		this.jointsCount = 0;
		this.numContacts = 0;
	}

	addBody(setup) {
		let body = new World.Body(this, setup);
		this.bodies.push(body);
		this.bodyCount = this.bodies.length;
		return body;
	}

	addJoint(setup) {
		let joint = new World.Joint(this, setup);
		this.joints.push(joint);
		this.jointsCount = this.joints.length;
		return joint;
	}

	addShape(setup) {
		const shape = new World.Shape(setup);
		this.shapes[setup.id] = shape;
		return shape;
	}

	removeBody(body) {
		for (let i = 0; i < this.bodyCount; ++i) {
			if (this.bodies[i] === body) {
				this.bodies.splice(i, 1);
				for (let j = 0; j < this.jointsCount; ++j) {
					const joint = this.joints[j];
					if (joint.bA === body || joint.bB === body) {
						this.joints.splice(j, 1);
						this.jointsCount--;
						j--;
					}
				}
				break;
			}
		}
		this.bodyCount = this.bodies.length;
	}

	step() {
		let i, j;
		this.contacts.length = 0;
		this.numContacts = 0;
		// broadphase
		this.broadPhase();
		// integrate forces (gravity)
		for (i = 0; i < this.bodyCount; ++i) {
			this.bodies[i].integrateGravity();
		}
		// Perform joints pre-steps
		for (i = 0; i < this.jointsCount; ++i) {
			this.joints[i].preStep();
		}
		// Perform iterations
		for (j = 0; j < this.iterations; ++j) {
			for (i = 0; i < this.numContacts; ++i) {
				this.contacts[i].applyImpulse();
			}
			for (i = 0; i < this.jointsCount; ++i) {
				this.joints[i].applyImpulse();
			}
		}
		// integrate velocities
		for (i = 0; i < this.bodyCount; ++i) {
			this.bodies[i].integrate();
		}
		// draw bodies
		for (i = 0; i < this.bodyCount; ++i) {
			this.bodies[i].draw();
		}
	}

	broadPhase() {
		let i, j, bi, bj, dx, dy, d;
		for (i = 0; i < this.bodyCount - 1; ++i) {
			bi = this.bodies[i];
			for (j = i + 1; j < this.bodyCount; ++j) {
				bj = this.bodies[j];
				if (bi.iM || bj.iM) {
					// AABB collision
					if (bi.overlapping(bj)) {
						// OBB collision and update contact points
						if (!bi.nocontact.includes(bj)) {
							this.collide(bi, bj);
						}
					}
				}
			}
		}
	}

	// Box vertex and edge numbering:
	//
	//        ^ y
	//        |
	//        e1
	//   v2 ------ v1
	//    |        |
	// e2 |        | e4  --> x
	//    |        |
	//   v3 ------ v4
	//        e3

	// The normal points from A to B
	collide(bA, bB) {
		// Setup
		let dpx = bB.px - bA.px;
		let dpy = bB.py - bA.py;
		let dax = bA.cos * dpx + bA.sin * dpy;
		let day = -bA.sin * dpx + bA.cos * dpy;
		let dbx = bB.cos * dpx + bB.sin * dpy;
		let dby = -bB.sin * dpx + bB.cos * dpy;
		let m00 = Math.abs(bA.cos * bB.cos + bA.sin * bB.sin);
		let m01 = Math.abs(-bA.sin * bB.cos + bA.cos * bB.sin);
		let m10 = Math.abs(-bA.cos * bB.sin + bA.sin * bB.cos);
		let m11 = Math.abs(bA.sin * bB.sin + bA.cos * bB.cos);
		// Box A faces
		let fAx = Math.abs(dax) - bA.hw - (m00 * bB.hw + m10 * bB.hh);
		let fAy = Math.abs(day) - bA.hh - (m01 * bB.hw + m11 * bB.hh);
		if (fAx > 0.0 || fAy > 0.0) return;
		// Box B faces
		let fBx = Math.abs(dbx) - bB.hw - (m00 * bA.hw + m01 * bA.hh);
		let fBy = Math.abs(dby) - bB.hh - (m10 * bA.hw + m11 * bA.hh);
		if (fBx > 0.0 || fBy > 0.0) return;
		// Find best axis
		let nx, ny;
		// Box A faces
		let axis = 0;
		let separation = fAx;
		if (dax > 0.0) {
			nx = bA.cos;
			ny = bA.sin;
		} else {
			nx = -bA.cos;
			ny = -bA.sin;
		}
		if (fAy > 0.95 * separation + 0.01 * bA.hh) {
			axis = 1;
			separation = fAy;
			if (day > 0.0) {
				nx = -bA.sin;
				ny = bA.cos;
			} else {
				nx = bA.sin;
				ny = -bA.cos;
			}
		}
		// Box B faces
		if (fBx > 0.95 * separation + 0.01 * bB.hw) {
			axis = 2;
			separation = fBx;
			if (dbx > 0.0) {
				nx = bB.cos;
				ny = bB.sin;
			} else {
				nx = -bB.cos;
				ny = -bB.sin;
			}
		}
		if (fBy > 0.95 * separation + 0.01 * bB.hh) {
			axis = 3;
			separation = fBy;
			if (dby > 0.0) {
				nx = -bB.sin;
				ny = bB.cos;
			} else {
				nx = bB.sin;
				ny = -bB.cos;
			}
		}
		// Setup clipping plane data based on the separating axis
		let fnx, fny, snx, sny, front, negSide, posSide, side;
		switch (axis) {
			case 0:
				fnx = nx;
				fny = ny;
				front = bA.px * fnx + bA.py * fny + bA.hw;
				snx = -bA.sin;
				sny = bA.cos;
				side = bA.px * snx + bA.py * sny;
				negSide = -side + bA.hh;
				posSide = side + bA.hh;
				this.computeIncidentEdge(bB, fnx, fny);
				break;
			case 1:
				fnx = nx;
				fny = ny;
				front = bA.px * fnx + bA.py * fny + bA.hh;
				snx = bA.cos;
				sny = bA.sin;
				side = bA.px * snx + bA.py * sny;
				negSide = -side + bA.hw;
				posSide = side + bA.hw;
				this.computeIncidentEdge(bB, fnx, fny);
				break;
			case 2:
				fnx = -nx;
				fny = -ny;
				front = bB.px * fnx + bB.py * fny + bB.hw;
				snx = -bB.sin;
				sny = bB.cos;
				side = bB.px * snx + bB.py * sny;
				negSide = -side + bB.hh;
				posSide = side + bB.hh;
				this.computeIncidentEdge(bA, fnx, fny);
				break;
			case 3:
				fnx = -nx;
				fny = -ny;
				front = bB.px * fnx + bB.py * fny + bB.hh;
				snx = bB.cos;
				sny = bB.sin;
				side = bB.px * snx + bB.py * sny;
				negSide = -side + bB.hw;
				posSide = side + bB.hw;
				this.computeIncidentEdge(bA, fnx, fny);
				break;
		}
		// Clip to box side 1
		if (this.clipSegmentToLine(-snx, -sny, negSide) < 2) return;
		// Clip to negative box side 1
		this.ie1x = this.oe1x;
		this.ie1y = this.oe1y;
		this.ie2x = this.oe2x;
		this.ie2y = this.oe2y;
		if (this.clipSegmentToLine(snx, sny, posSide) < 2) return;
		// Now oe contains the clipping points
		let friction = Math.sqrt(bA.friction * bB.friction);
		separation = fnx * this.oe1x + fny * this.oe1y - front;
		if (separation < 0) {
			this.contacts.push(new World.Contact(
				this,
				bA,
				bB,
				this.oe1x - fnx * separation,
				this.oe1y - fny * separation,
				nx,
				ny,
				separation,
				friction
			));
			this.numContacts++;
		}
		separation = fnx * this.oe2x + fny * this.oe2y - front;
		if (separation < 0) {
			this.contacts.push(new World.Contact(
				this,
				bA,
				bB,
				this.oe2x - fnx * separation,
				this.oe2y - fny * separation,
				nx,
				ny,
				separation,
				friction
			));
			this.numContacts++;
		}
	}

	clipSegmentToLine(nx, ny, offset) {
		// Start with no output points
		let numOut = 0;
		// Calculate the distance of end points to the line
		let distance0 = nx * this.ie1x + ny * this.ie1y - offset;
		let distance1 = nx * this.ie2x + ny * this.ie2y - offset;
		// If the points are behind the plane
		if (distance0 <= 0.0) {
			this.oe1x = this.ie1x;
			this.oe1y = this.ie1y;
			numOut++;
		}
		if (distance1 <= 0.0) {
			if (numOut === 0) {
				this.oe1x = this.ie2x;
				this.oe1y = this.ie2y;
			} else {
				this.oe2x = this.ie2x;
				this.oe2y = this.ie2y;
			}
			numOut++;
		}
		// If the points are on different sides of the plane
		if (distance0 * distance1 < 0.0) {
			// Find intersection point of edge and plane
			let interp = distance0 / (distance0 - distance1);
			this.oe2x = this.ie1x + interp * (this.ie2x - this.ie1x);
			this.oe2y = this.ie1y + interp * (this.ie2y - this.ie1y);
			numOut++;
		}
		return numOut;
	}

	computeIncidentEdge(b, nx, ny) {
		// The normal is from the reference box. Convert it
		// to the incident boxe's frame and flip sign.
		let nrx = -(b.cos * nx + b.sin * ny);
		let nry = -(-b.sin * nx + b.cos * ny);
		if (Math.abs(nrx) > Math.abs(nry)) {
			if (nrx >= 0.0) {
				this.ie1x = b.hw;
				this.ie1y = -b.hh;
				this.ie2x = b.hw;
				this.ie2y = b.hh;
			} else {
				this.ie1x = -b.hw;
				this.ie1y = b.hh;
				this.ie2x = -b.hw;
				this.ie2y = -b.hh;
			}
		} else {
			if (nry >= 0.0) {
				this.ie1x = b.hw;
				this.ie1y = b.hh;
				this.ie2x = -b.hw;
				this.ie2y = b.hh;
			} else {
				this.ie1x = -b.hw;
				this.ie1y = -b.hh;
				this.ie2x = b.hw;
				this.ie2y = -b.hh;
			}
		}
		let x, y;
		x = this.ie1x;
		y = this.ie1y;
		this.ie1x = b.px + b.cos * x - b.sin * y;
		this.ie1y = b.py + b.sin * x + b.cos * y;
		x = this.ie2x;
		y = this.ie2y;
		this.ie2x = b.px + b.cos * x - b.sin * y;
		this.ie2y = b.py + b.sin * x + b.cos * y;
	}
}

World.Contact = class {

	constructor(world, bA, bB, px, py, nx, ny, separation, friction) {
		this.bA = bA;
		this.bB = bB;
		this.nx = nx;
		this.ny = ny;
		this.separation = separation;
		this.friction = friction;
		this.r1x = px - bA.px;
		this.r1y = py - bA.py;
		this.r2x = px - bB.px;
		this.r2y = py - bB.py;
		this.Pn = 0.0; // accumulated normal impulse
		this.Pt = 0.0; // accumulated tangent impulse
		// Precompute normal mass, tangent mass, and bias.
		let rn1 = this.r1x * nx + this.r1y * ny;
		let rn2 = this.r2x * nx + this.r2y * ny;
		let rt1 = this.r1x * ny - this.r1y * nx;
		let rt2 = this.r2x * ny - this.r2y * nx;
		let r11 = this.r1x * this.r1x + this.r1y * this.r1y;
		let r22 = this.r2x * this.r2x + this.r2y * this.r2y;
		this.massNormal = 1.0 / (this.bA.iM + this.bB.iM +
			bA.iI * (r11 - rn1 * rn1) +
			bB.iI * (r22 - rn2 * rn2)
		);
		this.massTangent = -1.0 / (this.bA.iM + this.bB.iM +
			bA.iI * (r11 - rt1 * rt1) +
			bB.iI * (r22 - rt2 * rt2)
		);
		this.bias = -0.2 * world.invDT * Math.min(0.0, separation + 0.01);
	}

	applyImpulse() {
		let px, py, Pn, Pt, p;
		// Clamp the accumulated impulse
		Pn = this.Pn;
		this.Pn = Math.max(Pn + this.massNormal * (-(
			(this.bB.vx - this.bB.va * this.r2y - this.bA.vx + this.bA.va * this.r1y) * this.nx +
			(this.bB.vy + this.bB.va * this.r2x - this.bA.vy - this.bA.va * this.r1x) * this.ny
		) + this.bias), 0.0);
		// Apply normal impulse
		p = this.Pn - Pn;
		px = this.nx * p;
		py = this.ny * p;
		this.bA.applyImpulse(-px, -py, -(this.r1x * py - this.r1y * px));
		this.bB.applyImpulse(px, py, (this.r2x * py - this.r2y * px))
		// Compute friction impulse
		Pt = this.Pt;
		this.Pt = Math.min(this.friction * this.Pn,
			Math.max(-this.friction * this.Pn, Pt + this.massTangent * (
				(this.bB.vx - this.bB.va * this.r2y - this.bA.vx + this.bA.va * this.r1y) * this.ny -
				(this.bB.vy + this.bB.va * this.r2x - this.bA.vy - this.bA.va * this.r1x) * this.nx)
			)
		);
		// Apply friction impulse
		p = this.Pt - Pt;
		px = this.ny * p;
		py = -this.nx * p;
		this.bA.applyImpulse(-px, -py, -(this.r1x * py - this.r1y * px));
		this.bB.applyImpulse(px, py, (this.r2x * py - this.r2y * px));
	}
}

World.Body = class {

	constructor(world, setup) {
		this.world = world;
		let w = setup.w || 1.0;
		let h = setup.h || 1.0;
		this.px = setup.x || 0.0;
		this.py = setup.y || 0.0;
		this.vx = setup.vx || 0.0;
		this.vy = setup.vy || 0.0;
		this.hw = w * 0.5;
		this.hh = h * 0.5;
		this.va = setup.angularVelocity || 0.0;
		this.ra = setup.rotation || 0.0;
		this.cos = Math.cos(this.ra);
		this.sin = Math.sin(this.ra);
		this.xmin = 0.0;
		this.xmax = 0.0;
		this.ymin = 0.0;
		this.ymax = 0.0;
		this.x1 = 0.0;
		this.x2 = 0.0;
		this.x3 = 0.0;
		this.x4 = 0.0;
		this.y1 = 0.0;
		this.y2 = 0.0;
		this.y3 = 0.0;
		this.y4 = 0.0;
		this.chw = 0.0;
		this.shw = 0.0;
		this.chh = 0.0;
		this.shh = 0.0;
		this.nocontact = [];
		this.shape = setup.shape ?
			world.shapes[setup.shape] :
			new World.Shape({});
		this.ctx = ctx;
		this.updateBox();
		this.friction = setup.friction === undefined
			? world.friction
			: setup.friction;
		let mass = setup.mass || Infinity;
		this.color = setup.color || "#FFF";
		this.visible = setup.visible === undefined ? true : setup.visible;
		if (mass < Infinity) {
			this.iM = 1.0 / mass;
			this.iI = 1.0 / (mass * (w * w + h * h) / 12);
		} else {
			this.iM = 0.0;
			this.iI = 0.0;
		}
		this.dt = world.timeStep;
		this.gravity = setup.gravity || world.gravity;
	}

	applyImpulse(px, py, pa) {
		this.vx += this.iM * px;
		this.vy += this.iM * py;
		this.va += this.iI * pa;
	}

	integrateGravity() {
		if (!this.iM) return;
		this.vy += this.gravity * this.iM * this.dt;
	}

	integrate() {
		if (!this.iM) return;
		this.px += this.vx * this.dt;
		this.py += this.vy * this.dt;
		this.ra += this.va * this.dt;
		this.cos = Math.cos(this.ra);
		this.sin = Math.sin(this.ra);
		this.updateBox();
		if (this.py > this.world.maxy) {
			this.world.removeBody(this);
		}
	}

	updateBox() {
		this.chw = this.cos * this.hw;
		this.shw = this.sin * this.hw;
		this.chh = this.cos * this.hh;
		this.shh = this.sin * this.hh;
		this.xmin = 20000;
		this.xmax = -20000;
		this.x1 = this.px - this.chw + this.shh;
		this.x2 = this.px + this.chw + this.shh;
		this.x3 = this.px + this.chw - this.shh;
		this.x4 = this.px - this.chw - this.shh;
		if (this.x1 < this.xmin) this.xmin = this.x1;
		if (this.x1 > this.xmax) this.xmax = this.x1;
		if (this.x2 < this.xmin) this.xmin = this.x2;
		if (this.x2 > this.xmax) this.xmax = this.x2;
		if (this.x3 < this.xmin) this.xmin = this.x3;
		if (this.x3 > this.xmax) this.xmax = this.x3;
		if (this.x4 < this.xmin) this.xmin = this.x4;
		if (this.x4 > this.xmax) this.xmax = this.x4;
		this.ymin = 20000;
		this.ymax = -20000;
		this.y1 = this.py - this.shw - this.chh;
		this.y2 = this.py + this.shw - this.chh;
		this.y3 = this.py + this.shw + this.chh;
		this.y4 = this.py - this.shw + this.chh;
		if (this.y1 < this.ymin) this.ymin = this.y1;
		if (this.y1 > this.ymax) this.ymax = this.y1;
		if (this.y2 < this.ymin) this.ymin = this.y2;
		if (this.y2 > this.ymax) this.ymax = this.y2;
		if (this.y3 < this.ymin) this.ymin = this.y3;
		if (this.y3 > this.ymax) this.ymax = this.y3;
		if (this.y4 < this.ymin) this.ymin = this.y4;
		if (this.y4 > this.ymax) this.ymax = this.y4;
	}

	overlapping(b) {
		if ((this.xmax < b.xmin) || (this.xmin > b.xmax)) return false;
		if ((this.ymax < b.ymin) || (this.ymin > b.ymax)) return false;
		return true;
	}

	draw() {
		if (this.shape.loaded) {
			this.ctx.save();
			this.ctx.translate(this.px, this.py);
			this.ctx.rotate(this.ra);
			this.shape.draw();
			this.ctx.restore();
		}
		if (this.world.DEBUG) {
			this.ctx.beginPath();
			this.ctx.moveTo(this.x1, this.y1);
			this.ctx.lineTo(this.x2, this.y2);
			this.ctx.lineTo(this.x3, this.y3);
			this.ctx.lineTo(this.x4, this.y4);
			this.ctx.closePath();
			this.ctx.strokeStyle = "#000";
			this.ctx.stroke();
		}
	}

	addJoint(setup) {
		setup.b1 = setup.body;
		setup.b2 = this;
		this.world.addJoint(setup);
		return this;
	}

	noContact(list) {
		for (const b of list) {
			if (!this.nocontact.includes(b)) this.nocontact.push(b);
			if (!b.nocontact.includes(this)) b.nocontact.push(this);
		}
	}
}

World.Joint = class {

	constructor(world, setup) {
		this.bA = setup.b1;
		this.bB = setup.b2;
		let c, s, x, y;
		c = this.bA.cos;
		s = this.bA.sin;
		x = setup.ax - this.bA.px;
		y = setup.ay - this.bA.py;
		this.a1x = c * x + s * y;
		this.a1y = -s * x + c * y;
		c = this.bB.cos;
		s = this.bB.sin;
		x = setup.ax - this.bB.px;
		y = setup.ay - this.bB.py;
		this.a2x = c * x + s * y;
		this.a2y = -s * x + c * y;
		this.m00 = 0.0;
		this.m01 = 0.0;
		this.m11 = 0.0;
		this.r1x = 0.0;
		this.r1y = 0.0;
		this.r2x = 0.0;
		this.r2y = 0.0;
		this.bsx = 0.0;
		this.bsy = 0.0;
		this.aix = 0.0; // accumulated impulse
		this.aiy = 0.0;
		this.biasFactor = -0.2 * world.invDT;
		this.softness = setup.softness || 0.0;
		this.iM = this.bA.iM + this.bB.iM + this.softness;
	}

	preStep() {
		// Pre-compute anchors, mass matrix, and bias.
		this.r1x = this.bA.cos * this.a1x - this.bA.sin * this.a1y;
		this.r1y = this.bA.sin * this.a1x + this.bA.cos * this.a1y;
		this.r2x = this.bB.cos * this.a2x - this.bB.sin * this.a2y;
		this.r2y = this.bB.sin * this.a2x + this.bB.cos * this.a2y;
		let Km00 = this.iM +
			this.bA.iI * this.r1y * this.r1y +
			this.bB.iI * this.r2y * this.r2y;
		let Km01 = -this.bA.iI * this.r1x * this.r1y - this.bB.iI * this.r2x * this.r2y;
		let Km11 = this.iM +
			this.bA.iI * this.r1x * this.r1x +
			this.bB.iI * this.r2x * this.r2x;
		let det = 1.0 / (Km00 * Km11 - Km01 * Km01);
		this.m00 = det * Km11;
		this.m01 = -det * Km01;
		this.m11 = det * Km00;
		this.bsx = (this.bB.px + this.r2x - (this.bA.px + this.r1x)) * this.biasFactor;
		this.bsy = (this.bB.py + this.r2y - (this.bA.py + this.r1y)) * this.biasFactor;
		// Apply accumulated impulse.
		this.bA.vx -= this.aix * this.bA.iM;
		this.bA.vy -= this.aiy * this.bA.iM;
		this.bA.va -= this.bA.iI * (this.r1x * this.aiy - this.r1y * this.aix);
		this.bB.vx += this.aix * this.bB.iM;
		this.bB.vy += this.aiy * this.bB.iM;
		this.bB.va += this.bB.iI * (this.r2x * this.aiy - this.r2y * this.aix);
	}

	applyImpulse() {
		let bx = this.bsx -
			(this.bB.vx + -this.bB.va * this.r2y - this.bA.vx - -this.bA.va * this.r1y) -
			this.aix * this.softness;
		let by = this.bsy -
			(this.bB.vy + this.bB.va * this.r2x - this.bA.vy - this.bA.va * this.r1x) -
			this.aiy * this.softness;
		let ix = this.m00 * bx + this.m01 * by;
		let iy = this.m01 * bx + this.m11 * by;
		this.bA.vx -= ix * this.bA.iM;
		this.bA.vy -= iy * this.bA.iM;
		this.bA.va -= this.bA.iI * (this.r1x * iy - this.r1y * ix);
		this.bB.vx += ix * this.bB.iM;
		this.bB.vy += iy * this.bB.iM;
		this.bB.va += this.bB.iI * (this.r2x * iy - this.r2y * ix);
		this.aix += ix;
		this.aiy += iy;
	}
}

World.Shape = class {

	constructor(setup) {
		this.ctx = ctx;
		this.width = setup.w || 0.0;
		this.height = setup.h || 0.0;
		this.offset = setup.offset || 0.0;
		this.loaded = false;
		this.texture = null;
		if (!setup.src) return;
		const source = new Image();
		source.onload = e => {
			this.loaded = true;
			this.texture = document.createElement("canvas");
			this.texture.width = source.height;
			this.texture.height = source.width;
			const ict = this.texture.getContext("2d");
			ict.translate(source.height / 2, source.width / 2);
			ict.rotate(setup.rotate);
			ict.drawImage(source, -source.width / 2, -source.height / 2);
		};
		source.src = setup.src;
	}

	draw() {
		this.ctx.drawImage(
			this.texture,
			-this.width * 0.5,
			-this.height * 0.5 + this.offset,
			this.width,
			this.height
		);
	}

	noContact(list) {
		for (const b of list) {
			if (!this.nocontact.includes(b)) this.nocontact.push(b);
			if (!b.nocontact.includes(this)) b.nocontact.push(this);
		}
	}
}


let ground = null;
let groundWidth = 0;
const humans = [];
let ctx;
let frame = 0;


class Canvas {
	constructor(canvas) {
		this.elem = canvas;

		this.elem.setAttribute('style', 'border-bottom: #000 1px solid;');
		this.ctx = this.elem.getContext("2d");

		this.resize();
		window.addEventListener("resize", () => this.resize(), false);
	}


	resize() {
		this.width = this.elem.width = this.elem.offsetWidth;
		this.height = this.elem.height = this.elem.offsetHeight;
		if (ground) {
			groundWidth = this.width;
			ground.px = this.width / 2
			ground.hw = groundWidth * 0.3;
			ground.updateBox();
		}
	}
}
///////////////////////////////////////////////







export default class Bodies {
	constructor(_canvas) {
		this._canvas = _canvas

	}


	run() {

		const human = (x, y) => {
			const human = [];
			const armleft = world.addBody({
				x: x,
				y: y + s * 2,
				w: s * 0.75,
				h: s * 2,
				mass: 1,
				shape: "arm"
			});
			human.push(armleft);
			const forearmleft = world.addBody({
				x: x,
				y: y + s * 4,
				w: s * 0.5,
				h: s * 2.5,
				mass: 1,
				shape: "forearm"
			});
			human.push(forearmleft);
			const legleft = world.addBody({
				x: x - s * 0.25,
				y: y + s * 5.25,
				w: s * 1,
				h: s * 2.5,
				mass: 2,
				shape: "leg"
			});
			human.push(legleft);
			const forelegleft = world.addBody({
				x: x - s * 0.25,
				y: y + s * 7.5,
				w: s * 1,
				h: s * 3,
				mass: 1,
				shape: "foreleg"
			});
			human.push(forelegleft);
			const pleft = world.addBody({
				x: x + s * 0.25,
				y: y + s * 6,
				w: s * 0.01,
				h: s * 1,
				mass: 2
			}).addJoint({ body: legleft, ax: x + s * 0.25, ay: y + s * 6, softness: 0.5 });
			human.push(pleft);
			const legright = world.addBody({
				x: x - 1,
				y: y + s * 5.25,
				w: s * 1,
				h: s * 2.5,
				mass: 2,
				shape: "leg"
			});
			human.push(legright);
			const forelegright = world.addBody({
				x: x - 1,
				y: y + s * 7.5,
				w: s * 1,
				h: s * 3,
				mass: 1,
				shape: "foreleg"
			});
			human.push(forelegright);
			const pright = world
				.addBody({
					x: x + s * 0.25,
					y: y + s * 6,
					w: s * 0.01,
					h: s * 1,
					mass: 2
				})
				.addJoint({ body: legright, ax: x + s * 0.25, ay: y + s * 6, softness: 0.5 });
			human.push(pright);
			const head = world.addBody({
				x: x,
				y: y,
				w: s,
				h: s * 1.5,
				mass: 1,
				gravity: -50,
				friction: 0.001,
				shape: "head"
			});
			human.push(head);
			const torso = world.addBody({
				x: x,
				y: y + s * 2.75,
				w: s * 1.5,
				h: s * 3.5,
				mass: 5,
				angularVelocity: Math.random() * 50 - 25,
				shape: "torso"
			});
			human.push(torso);
			const tright = world.addBody({
				x: x - s * 0.75,
				y: y + s * 3.75,
				w: s * 0.2,
				h: s * 1,
				mass: 2
			}).addJoint({ body: torso, ax: x - s * 0.75, ay: y + s * 3.75, softness: 0.5 });
			human.push(tright);
			const armright = world.addBody({
				x: x + 1,
				y: y + s * 2.3,
				w: s * 1,
				h: s * 2,
				mass: 1,
				shape: "arm"
			});
			human.push(armright);
			const forearmright = world.addBody({
				x: x + 1,
				y: y + s * 4,
				w: s * 0.5,
				h: s * 2.6,
				mass: 1,
				shape: "forearm"
			});
			human.push(forearmright);
			torso.addJoint({ body: head, ax: x, ay: y + s, softness: 0.5 });
			armright.addJoint({ body: torso, ax: x + 1, ay: y + s, softness: 0.5 });
			forearmright.addJoint({ body: armright, ax: x + 1, ay: y + s * 3, softness: 0.5 });
			armleft.addJoint({ body: torso, ax: x, ay: y + s, softness: 0.5 });
			forearmleft.addJoint({ body: armleft, ax: x, ay: y + s * 3, softness: 0.5 });
			legright.addJoint({ body: torso, ax: x - 1, ay: y + s * 4.25, softness: 0.5 });
			forelegright.addJoint({ body: legright, ax: x - 1, ay: y + s * 6.26, softness: 0.5 });
			legleft.addJoint({ body: torso, ax: x - s * 0.25, ay: y + s * 4.25, softness: 0.5 });
			forelegleft.addJoint({ body: legleft, ax: x - s * 0.25, ay: y + s * 6.26, softness: 0.5 });
			armright.noContact([torso, tright, head, forearmright, forearmleft])
			forearmright.noContact([tright, forearmleft]);
			armleft.noContact([head, torso, tright, armright, forearmleft, forearmright]);
			legleft.noContact([torso, legright, forelegleft, forelegright, pright]);
			legright.noContact([torso, forelegright, forelegleft, pleft]);
			forelegleft.noContact([legleft, pright, forelegright]);
			forelegright.noContact([pleft]);
			return human;
		};


		const canvas = new Canvas(this._canvas);
		ctx = canvas.ctx;
		const width = canvas.width, height = canvas.height;

		const world = new World({
			gravity: 90,
			iterations: 9,
			timeStep: 1 / 30,
			friction: 2.5,
			maxy: height * 4
		});
		ground = world.addBody({
			x: width / 2,
			y: height + 400,
			w: Math.min(width, height),
			h: 800,
			mass: Infinity
		});

		var run = () => {
			requestAnimationFrame(run);
			if (frame++ % 120 === 0) {
				humans.push(human(
					canvas.width / 2,
					-canvas.height / 2.5
				));
			}
			ground.py += (canvas.height + 400 - ground.py) * 0.05;
			ctx.fillStyle = "#fff";
			ctx.fillRect(0, 0, canvas.width, canvas.height);
			ctx.fillStyle = "#fff";
			ctx.fillRect((canvas.width - groundWidth) * 0.5, 0, groundWidth, canvas.height);
			world.step();
		}

		const s = height / 30;
		world.addShape({
			id: "head",
			w: s * 1.6,
			h: s * 2.1,
			offset: s * 0.2,
			rotate: -Math.PI / 2,
			src: "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAGwAAABTCAYAAAB6SuK1AAAAGXRFWHRTb2Z0d2FyZQBBZG9iZSBJbWFnZVJlYWR5ccllPAAAA4ZpVFh0WE1MOmNvbS5hZG9iZS54bXAAAAAAADw/eHBhY2tldCBiZWdpbj0i77u/IiBpZD0iVzVNME1wQ2VoaUh6cmVTek5UY3prYzlkIj8+IDx4OnhtcG1ldGEgeG1sbnM6eD0iYWRvYmU6bnM6bWV0YS8iIHg6eG1wdGs9IkFkb2JlIFhNUCBDb3JlIDUuNi1jMTQwIDc5LjE2MDQ1MSwgMjAxNy8wNS8wNi0wMTowODoyMSAgICAgICAgIj4gPHJkZjpSREYgeG1sbnM6cmRmPSJodHRwOi8vd3d3LnczLm9yZy8xOTk5LzAyLzIyLXJkZi1zeW50YXgtbnMjIj4gPHJkZjpEZXNjcmlwdGlvbiByZGY6YWJvdXQ9IiIgeG1sbnM6eG1wTU09Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC9tbS8iIHhtbG5zOnN0UmVmPSJodHRwOi8vbnMuYWRvYmUuY29tL3hhcC8xLjAvc1R5cGUvUmVzb3VyY2VSZWYjIiB4bWxuczp4bXA9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC8iIHhtcE1NOk9yaWdpbmFsRG9jdW1lbnRJRD0ieG1wLmRpZDo0YWJjMTA3NC1jNDFjLTRkYzctOWFiOC02MTI4NGZjNDc2ODAiIHhtcE1NOkRvY3VtZW50SUQ9InhtcC5kaWQ6RjIzNjg3OTI1RDI0MTFFODkxMzI5NzRCQzlFMkZENzgiIHhtcE1NOkluc3RhbmNlSUQ9InhtcC5paWQ6RjIzNjg3OTE1RDI0MTFFODkxMzI5NzRCQzlFMkZENzgiIHhtcDpDcmVhdG9yVG9vbD0iQWRvYmUgUGhvdG9zaG9wIENDIDIwMTggKE1hY2ludG9zaCkiPiA8eG1wTU06RGVyaXZlZEZyb20gc3RSZWY6aW5zdGFuY2VJRD0ieG1wLmlpZDphNWFiY2I3ZS1kMWVjLTQ4YjMtYWI1Ny1mYjRmNTI4OGZhOWIiIHN0UmVmOmRvY3VtZW50SUQ9ImFkb2JlOmRvY2lkOnBob3Rvc2hvcDo5OTMyNDZkNy05YTI4LTBmNDktOWMx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"
		});
		world.addShape({
			id: "torso",
			w: s * 2.2,
			h: s * 3.75,
			offset: -s * 0.1,
			rotate: Math.PI / 2,
			src: "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAPwAAABmCAYAAADvVvxCAAAAGXRFWHRTb2Z0d2FyZQBBZG9iZSBJbWFnZVJlYWR5ccllPAAAA4ZpVFh0WE1MOmNvbS5hZG9iZS54bXAAAAAAADw/eHBhY2tldCBiZWdpbj0i77u/IiBpZD0iVzVNME1wQ2VoaUh6cmVTek5UY3prYzlkIj8+IDx4OnhtcG1ldGEgeG1sbnM6eD0iYWRvYmU6bnM6bWV0YS8iIHg6eG1wdGs9IkFkb2JlIFhNUCBDb3JlIDUuNi1jMTQwIDc5LjE2MDQ1MSwgMjAxNy8wNS8wNi0wMTowODoyMSAgICAgICAgIj4gPHJkZjpSREYgeG1sbnM6cmRmPSJodHRwOi8vd3d3LnczLm9yZy8xOTk5LzAyLzIyLXJkZi1zeW50YXgtbnMjIj4gPHJkZjpEZXNjcmlwdGlvbiByZGY6YWJvdXQ9IiIgeG1sbnM6eG1wTU09Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC9tbS8iIHhtbG5zOnN0UmVmPSJodHRwOi8vbnMuYWRvYmUuY29tL3hhcC8xLjAvc1R5cGUvUmVzb3VyY2VSZWYjIiB4bWxuczp4bXA9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC8iIHhtcE1NOk9yaWdpbmFsRG9jdW1lbnRJRD0ieG1wLmRpZDo1NzAyMWU1NS1hZDIwLTRjMDktOWJlOS1jNzVmZDc2N2ZiOTMiIHhtcE1NOkRvY3VtZW50SUQ9InhtcC5kaWQ6QUQzNEFBMDE2MDU2MTFFODkzQUI5NTY2MThBRkVFOUUiIHhtcE1NOkluc3RhbmNlSUQ9InhtcC5paWQ6QUQzNEFBMDA2MDU2MTFFODkzQUI5NTY2MThBRkVFOUUiIHhtcDpDcmVhdG9yVG9vbD0iQWRvYmUgUGhvdG9zaG9wIENDIDIwMTggKE1hY2ludG9zaCkiPiA8eG1wTU06RGVyaXZlZEZyb20gc3RSZWY6aW5zdGFuY2VJRD0ieG1wLmlpZDoyYTQ3Zjg2ZS1lYmIyLTQ3YWYtODdhMC1lYzY2NGY4NTQ3MmMiIHN0UmVmOmRvY3VtZW50SUQ9ImFkb2JlOmRvY2lkOnBob3Rvc2hvcDo0N2I1MGY1MC1jOTcwLTgwNGQtOTQ3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"
		});
		world.addShape({
			id: "arm",
			w: s * 1.3,
			h: s * 2.4,
			offset: -s * 0.35,
			rotate: Math.PI / 2,
			src: "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAIcAAABCCAYAAAB9wsUNAAAAGXRFWHRTb2Z0d2FyZQBBZG9iZSBJbWFnZVJlYWR5ccllPAAAAyhpVFh0WE1MOmNvbS5hZG9iZS54bXAAAAAAADw/eHBhY2tldCBiZWdpbj0i77u/IiBpZD0iVzVNME1wQ2VoaUh6cmVTek5UY3prYzlkIj8+IDx4OnhtcG1ldGEgeG1sbnM6eD0iYWRvYmU6bnM6bWV0YS8iIHg6eG1wdGs9IkFkb2JlIFhNUCBDb3JlIDUuNi1jMTQwIDc5LjE2MDQ1MSwgMjAxNy8wNS8wNi0wMTowODoyMSAgICAgICAgIj4gPHJkZjpSREYgeG1sbnM6cmRmPSJodHRwOi8vd3d3LnczLm9yZy8xOTk5LzAyLzIyLXJkZi1zeW50YXgtbnMjIj4gPHJkZjpEZXNjcmlwdGlvbiByZGY6YWJvdXQ9IiIgeG1sbnM6eG1wPSJodHRwOi8vbnMuYWRvYmUuY29tL3hhcC8xLjAvIiB4bWxuczp4bXBNTT0iaHR0cDovL25zLmFkb2JlLmNvbS94YXAvMS4wL21tLyIgeG1sbnM6c3RSZWY9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC9zVHlwZS9SZXNvdXJjZVJlZiMiIHhtcDpDcmVhdG9yVG9vbD0iQWRvYmUgUGhvdG9zaG9wIENDIDIwMTggKE1hY2ludG9zaCkiIHhtcE1NOkluc3RhbmNlSUQ9InhtcC5paWQ6NTQ2RDZDN0Q1RDE3MTFFODkxMzI5NzRCQzlFMkZENzgiIHhtcE1NOkRvY3VtZW50SUQ9InhtcC5kaWQ6NTQ2RDZDN0U1RDE3MTFFODkxMzI5NzRCQzlFMkZENzgiPiA8eG1wTU06RGVyaXZlZEZyb20gc3RSZWY6aW5zdGFuY2VJRD0ieG1wLmlpZDo1NDZENkM3QjVEMTcxMUU4OTEzMjk3NEJDOUUyRkQ3OCIgc3RSZWY6ZG9jdW1lbnRJRD0ieG1wLmRpZDo1NDZENkM3QzVEMTcxMUU4OTEzMjk3NEJDOUUyRkQ3OCIvPiA8L3JkZjpEZXNjcmlwdGlvbj4gPC9yZGY6UkRGPiA8L3g6eG1wbWV0YT4gPD94cGFja2V0IGVuZD0iciI/PjhuWwsAAAR9SURBVHja7J3Zb01RFMb3NVSpUPNQTy0hDTEmxJyGoEKk4oVI8NZ48OJBPOBF4r/wYkhEFDG80BA1pObUWEMQaoqhWqWlrW/Z6yQ3t/eotvece4bvS37aVCrt2p+z9157n7WMoSgXJSL4Ow0DY8BY/ThavzYU5IPBYBAYCHJBHshR8lz+ze+gRZHPf4JG8A3Ug6/gM/gE3ilvwFv9Gs3hkwaA8WBCCgVqiNyA/bxipDo1y5MUnoImmqN7P1shmA6mgRlgMhgXoSdeO3gN7oFb4A64DZ7r39EcKnnUzwcLwBw1RH5Mp/t6Nck1cAlU6RQWG3PI3F8CFqkhpoLeXAamVSu4q0a5CCrVQJEyx0SwBpSCuaAPx71b+g2ugDPgOHgc1l+kCOwCD3UeJZlHYrtbYx14ybaxXN3dxsHzjTaNebmOQaAkU8VB3b5xsLKLjMEhMC+bhpBF5DpQzQEJLNU6Rr4t+HuB9eARgx8aZKw26Nh5puWghsEOLfd1DDMqOauoYHAjQ4WOaY+1ydjDJQY1WnzWse2WhoDTDGLkkYTa8K4YYxKoZeBiw0s94OxUS8EXBix2NIAV/zpbKdXFSg6PMWIpudRUls4cs/Xkrx9jFGs1J9JsVeVoeARjQ6VmyvbTGFQ6c0jufRlDQjlKJH2Ue4zFDAmV+uRYTGNQbuYoYygoN3PMYSgotzWHHL4MYTiodOZoNR5fAKHCO63UMxSUmzmeMRSUmzmuMRSUmzmOMRSU24KUGVKqg5LfY/ho7PkKRXUwxwNj344qYlio5GnFEe9zUB0WpI6khtUqE4CqMlTW1eJ2h1TqVPVnfOJrDLDWLWXOoirxlVRJlAI7p9zMwXR6PPXK2Ls9Z9OtORzVMU6xkxhiFrjptiB1VMNYxUby4toWsNLYXJfpzBxVjFksdMLY2q77u7JDlUrArYavBrI+h4vOMIiRQ0pSZqSyzxIGMzLcMB7UBKtkYENLMzhsbMlwTyRH+CwbGS6kDulW41Md0u0MeCiqBO4xttWIr5LzlwMcgEDxy9gyGTuMrb7k2cD/j3I0g1bC1EDWJPdtLoDzSqC6JuTp9nYhx8lzSW8VacxzHVwGV41tDeb7lNEV9dcphu/WZk5Op6Y7eq4R6k5N8j07dQHEo/3OldzjTXq6pfZ5i2SPN6kdJh0SeOfUSrpGngMn1QzyRAh1d8ieStpv7tWES1x3Di/ANuPedjS8am/PyNQm++ujJl6Nd+Qi9kbQ1+vxyRaZMoejKeCIie6JbpMuyOf7+Z83KuZwJOuQfeB9BAwhC8pTYLOxHS19f7Jni8TfPxKeNYmUR+5qsNbYm0aDQjLbSvtxOXSsUGN8Dfm0373disfmSJZURJZrAFLOUjKtxSY4TY8bNNFUqdnH2zo1RmVNGHhzpGqUsc2GZxrbfVo++vGmnWQa5dLLLU06CbW6mI7qhiF05kinEbpeKVQKwEhj+4AIkqF1WpjL57m6JvhhbCNeeQLIhdkPxl6W/ai5hpeagBIaY7ibjIQ5qACZg0XiKFf9EWAAueOF054kGYsAAAAASUVORK5CYII="
		});
		world.addShape({
			id: "forearm",
			w: s * 1.2,
			h: s * 2.6,
			offset: s * 0,
			rotate: Math.PI / 2,
			src: "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAIIAAAA4CAYAAAAxUiDuAAAAGXRFWHRTb2Z0d2FyZQBBZG9iZSBJbWFnZVJlYWR5ccllPAAAA4ZpVFh0WE1MOmNvbS5hZG9iZS54bXAAAAAAADw/eHBhY2tldCBiZWdpbj0i77u/IiBpZD0iVzVNME1wQ2VoaUh6cmVTek5UY3prYzlkIj8+IDx4OnhtcG1ldGEgeG1sbnM6eD0iYWRvYmU6bnM6bWV0YS8iIHg6eG1wdGs9IkFkb2JlIFhNUCBDb3JlIDUuNi1jMTQwIDc5LjE2MDQ1MSwgMjAxNy8wNS8wNi0wMTowODoyMSAgICAgICAgIj4gPHJkZjpSREYgeG1sbnM6cmRmPSJodHRwOi8vd3d3LnczLm9yZy8xOTk5LzAyLzIyLXJkZi1zeW50YXgtbnMjIj4gPHJkZjpEZXNjcmlwdGlvbiByZGY6YWJvdXQ9IiIgeG1sbnM6eG1wTU09Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC9tbS8iIHhtbG5zOnN0UmVmPSJodHRwOi8vbnMuYWRvYmUuY29tL3hhcC8xLjAvc1R5cGUvUmVzb3VyY2VSZWYjIiB4bWxuczp4bXA9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC8iIHhtcE1NOk9yaWdpbmFsRG9jdW1lbnRJRD0ieG1wLmRpZDpjMTdhYjRhNC1mYmZmLTQxMDktODU1OS1jM2M5NmI1ZjIxZTUiIHhtcE1NOkRvY3VtZW50SUQ9InhtcC5kaWQ6RjIzNjg3OEU1RDI0MTFFODkxMzI5NzRCQzlFMkZENzgiIHhtcE1NOkluc3RhbmNlSUQ9InhtcC5paWQ6QThFRTREQTA1RDFGMTFFODkxMzI5NzRCQzlFMkZENzgiIHhtcDpDcmVhdG9yVG9vbD0iQWRvYmUgUGhvdG9zaG9wIENDIDIwMTggKE1hY2ludG9zaCkiPiA8eG1wTU06RGVyaXZlZEZyb20gc3RSZWY6aW5zdGFuY2VJRD0ieG1wLmlpZDoyZDFmMzIwNC0wOThhLTRhNTAtOTNlYi04Zjk5YmRiZjQ3MGMiIHN0UmVmOmRvY3VtZW50SUQ9ImFkb2JlOmRvY2lkOnBob3Rvc2hvcDo4Yzc1Mjc0ZS05OWIzLTIzNGEtYjQxYy05MGExZjg3YTdkOWUiLz4gPC9yZGY6RGVzY3JpcHRpb24+IDwvcmRmOlJERj4gPC94OnhtcG1ldGE+IDw/eHBhY2tldCBlbmQ9InIiPz56yzACAAAEJElEQVR42uyczWsTQRjGZ7VNqqC0iChEFD2oKOg18SqC3r0J6kEPIk3uPXj0KPjRmyLqoQf9AwT1nGKlbaQHrYr1uwp+1E2tsbXj+25m2+24aTbZbLIfzwM/uk0m65p5duZ9p/uOIaUUUHRlGMZqb9ftXOp/6wRr8FXGTtKBFyNZ7brwvcXKAM2OKtJo0UX0EnuIvQo+3k3MEdPEJwUffyS+EO/Vzwr6sMEed0zn9h3tV/qIsI84Qxwjdqmp47fWdpb4TMyo33cSW31cw1fNLHzuD5pZ2Dw/YQHRcgMsnY/dRSdN0fEl4lyI44ZfjlHFNg0b5QXxnHiVkNFFtnh0WZoV2AQP+LVUKiXz+bwsFovSNE0LPubX+D0tEAkbC8oM94nLxHniCLGDDR+zILBZXKcZe6q5xr9nMhk5NjYma4nf4zYhN0MtOFYpEXeJi8RpIkdsSogJap7HaYQFvttXM4Gt8fFxmU6no2qGWnCMUiRuEgPEceIAsS4mo4Bns8j+/n7pVdw2ZkaoxSIxpabNQaJAHFVB9NoYGcCC506OA0Q2m/V0hcPDwyKXyyU9aP9DvCQmHbxWmdSsyrRmVIBbacIE7c9C+B+uVCqCpgdPH+C2PT09yN8a03dlDo5VfihzzKqUmI9NoqzaDHTiAhteWZyfn0e3Nq6+BtpO+1yXaUrWmsHo6KjnD0xMTKBbg1VbTcDrCIxlhKGhIc8fbKQt1LTutSEkMPT1FSsl5NSwnkqlUhzTx7AujgWSIrplJvY6gvSyoMRGifCCEvCyjmDDC0uFQkGOjIzIcrlswcf8WgSWmIHfdQRMyRCeUILEUtYAQXhULdky7IwBRkioARAjQAamBkhoWaKBEQFaYQoYAYIRIBgBQowQefETTZsDHRH4cbV8Pi/4GUbTNC34mF/z+igbFLjSRHeTd7/Of5Fj3OsaEv3n5VXTBkddg0x4XUPiOr+mEVDXkDwDuBqB6xu9itviC+8YT4kTfjvfzQioawi3uJDmFnGHeCKqdREiiO2OUNcQPvFdf0VUS+3eSUev19kvyX/6iLqGjt/1V4lDxHriIHGDeMs3P3e+TeAri6hr6MhdzzvT8N4NPM/mRbUie66TF7WAuoa27M/Awz1XD/sut7cDvFbCGhSoawiCx8RJIiNavGNLUEZYsXUO6hrqYqpOvk1cIE4Rh9Xcvo3Y2OQScEeN0OhmWn9FtcR7UqUzj0S1vNuZe3arL8OpPi1D4XmxV2uzRazcgGKDS5vtLuftbcX0SDxT/y/++UZUd3JjvonlvQ44ZVrsdIARRPqoD1n7ibNieWcQjminiIfEdU4aIhCIGS4BsW7ubs3ckdqNrR1GgBJqBDyPAMEIEIwAwQiQm7r0wCPoNW0IIwIEI0ChnxrakaNCGBEgGAGCEaDI6Z8AAwDSisfyL6Z5CgAAAABJRU5ErkJggg=="
		});
		world.addShape({
			id: "leg",
			w: s * 1.5,
			h: s * 2.8,
			offset: 0,
			rotate: Math.PI / 2,
			src: "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAJoAAABJCAYAAADFVT+WAAAAGXRFWHRTb2Z0d2FyZQBBZG9iZSBJbWFnZVJlYWR5ccllPAAAA4ZpVFh0WE1MOmNvbS5hZG9iZS54bXAAAAAAADw/eHBhY2tldCBiZWdpbj0i77u/IiBpZD0iVzVNME1wQ2VoaUh6cmVTek5UY3prYzlkIj8+IDx4OnhtcG1ldGEgeG1sbnM6eD0iYWRvYmU6bnM6bWV0YS8iIHg6eG1wdGs9IkFkb2JlIFhNUCBDb3JlIDUuNi1jMTQwIDc5LjE2MDQ1MSwgMjAxNy8wNS8wNi0wMTowODoyMSAgICAgICAgIj4gPHJkZjpSREYgeG1sbnM6cmRmPSJodHRwOi8vd3d3LnczLm9yZy8xOTk5LzAyLzIyLXJkZi1zeW50YXgtbnMjIj4gPHJkZjpEZXNjcmlwdGlvbiByZGY6YWJvdXQ9IiIgeG1sbnM6eG1wTU09Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC9tbS8iIHhtbG5zOnN0UmVmPSJodHRwOi8vbnMuYWRvYmUuY29tL3hhcC8xLjAvc1R5cGUvUmVzb3VyY2VSZWYjIiB4bWxuczp4bXA9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC8iIHhtcE1NOk9yaWdpbmFsRG9jdW1lbnRJRD0ieG1wLmRpZDoxZWExODQxNi03NzNjLTRiYzEtODE1Yy03ZDdlN2I4MThkMmYiIHhtcE1NOkRvY3VtZW50SUQ9InhtcC5kaWQ6Qzc5OEFCRkY1RDE5MTFFODkxMzI5NzRCQzlFMkZENzgiIHhtcE1NOkluc3RhbmNlSUQ9InhtcC5paWQ6Qzc5OEFCRkU1RDE5MTFFODkxMzI5NzRCQzlFMkZENzgiIHhtcDpDcmVhdG9yVG9vbD0iQWRvYmUgUGhvdG9zaG9wIENDIDIwMTggKE1hY2ludG9zaCkiPiA8eG1wTU06RGVyaXZlZEZyb20gc3RSZWY6aW5zdGFuY2VJRD0ieG1wLmlpZDpjZTY4NTc1Ni0xYjNjLTQyYjctYmVkZi03ZGM5NzYyMGZhY2IiIHN0UmVmOmRvY3VtZW50SUQ9ImFkb2JlOmRvY2lkOnBob3Rvc2hvcDozNjczMDJiOS0wZDJiLTM5NGYtODE4Yi0yYzM0NDI1ODVjYWUiLz4gPC9yZGY6RGVzY3JpcHRpb24+IDwvcmRmOlJERj4gPC94OnhtcG1ldGE+IDw/eHBhY2tldCBlbmQ9InIiPz4Cfa0tAAAFUUlEQVR42uydS2gdVRzGT9JG6QvbmtioodrWRX0ubKulggup2upCBUFBAyJuVArFjYhLQbGCCurWjeBzoSi4iGgFTR9aX63VVunDNmprn9HGa0xt/D7nf2Aab+bO60zSe78f/Lg3yU24+c93z8w5c2ZO2+joqBMiNO0qgVDQhIImRBbaVAIREt8HmFrS37sM3gJXwKtgN5wBh+ABuBVugO/DPXApvBIugAvtsRPOhLPtA8B3eByegIft93bb4za4BdZSvr+5sAfOh11wjjk79tx/fTacbo/knDot/yk4aM+H4Z/2yPd7LGb860NwHxyAR9WiZdvt9sI1cEmWkJfUko7Ar+FG2AcPwkVwsQW3JxauaZOs7rVY6Absw7MD/mCPtWZr0fJu8Bvh89aSiXJha/kT3Am3m59ZAP9plRaNu5R18CEd31XfOCRsu/jPGMa/4V922MFd/H4L7lYL7c6qQ5slLPPge3CZtnlThPa47b6/gG/BfvjHRL8xHvvssjcom1Pusn+Db8PbrGNWKexB/qgN0XJy1/qNdfa6Qodshu3XVXi1dhyeugt2hAjaGyqyHOMR+AQ8r6yQ3a+iygTZq33OBtpzM89Gr1VQ2chBO47LdZbpNRVQZpTDJJdmCdk1dvCn4sms8pzvffVCNaXO916Gl2hMU+SAPdLbXTQRoS/phcv1qZQl+UJSi/a0i6bvCFEUHoIN2fjbaec6eQaAswbOUo1ESfDswrXsKMQn9PUqZKJkpthYW1u8RWP39GrVRgTgJn+Mxlmo61QPEYhTftd5q2ohArLSB+0G1UIEpNsfo/Eqo3NVDxGIUbZoFypkooqgaYBWhKadQbtOdRCBGWLQlqsOIjC/MmiLVAcRmJMMWrfqIAIzi8MbPPGp21eJkAy3K2Sikm6nSiAqYL+CJqqg39/wToiQrFLQRGg+hder1ylCctpU7mHVQwTiMRfN3P6vJTugeogAvASf8V8waLtUE1EyvCBlTfwbDNom1UWUyJPwkbGdTAatX7URJfExfLzeD9jr5AzbAdVIFIQ3Wr7CRTdg/h9s0X520V38hCjCw+OFzAeNrFedRAFehK8kvcAH7SPVSuTkXbi20Yv85XYXwb2qmcgIF5G706VYu8q3aLyL0Jeqm8jAm/AOl3KBtPg5ztdVO5GSp+DdLlpzKhXxuwmdb7tP3bpKjMfv8AEXrR2VifgdH7kSGpdF1AXFoh5cF5WL/35Sxh/TPWxlvTttP+rq31i7IVwY1i8OO5Y+FVeaXDZzQZGWKylonKimdQZaWy40t6qMXWRS0IhWTmlNuTJxrytxxnWjoGktqNbyK3hP3uOwIkEjWt2uuT0J34ErQ3ZV0wTNj/5qozSXnFHNOWMXVDEmkjZoM51WIG4GOVfsVWu9Kr3iLW3QSI+LzoVqg51562hyBWme9J42UaO8PmhtKV/Pe6h9UHRMRQRnO/zQRbMq1mc5FxkyaGPPdTaC91Hj3KNl2p6Thn0WLO+ku3QyT9DIdBddq/dgjt8VxeBKcbwYdwv8HG6Geyb7m84bNM/N8FkXnYQXJW8bOybeAb+H31qwvnPRLQbOrH+mYNCcDe7d66ILRZdkLGQZreGI9Yj77fjxoItWTl4ML7ZODK/wmj+RB8PjULPdHi8M4hVoey1Y3lrTfGpKCFqcy+FquMJFl1zNddEiGby66qh9KjfYQSqb+6Uumo600DoYfOyEs1y0TLJn0Lrmh+33dpvbbBeSdoN02rgRQ9cF5yTYYcM6HbHn9ThhYR+JPT+W4CEL1y/2/7RG81xy0IRIDNrUtINpQhRB90UTlfCvAAMACaQ/U5WwtjEAAAAASUVORK5CYII="
		});
		world.addShape({
			id: "foreleg",
			w: s * 2.2,
			h: s * 3,
			offset: s * 0,
			rotate: Math.PI / 2,
			src: "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAALsAAAB3CAYAAABbujGlAAAAGXRFWHRTb2Z0d2FyZQBBZG9iZSBJbWFnZVJlYWR5ccllPAAAA4ZpVFh0WE1MOmNvbS5hZG9iZS54bXAAAAAAADw/eHBhY2tldCBiZWdpbj0i77u/IiBpZD0iVzVNME1wQ2VoaUh6cmVTek5UY3prYzlkIj8+IDx4OnhtcG1ldGEgeG1sbnM6eD0iYWRvYmU6bnM6bWV0YS8iIHg6eG1wdGs9IkFkb2JlIFhNUCBDb3JlIDUuNi1jMTQwIDc5LjE2MDQ1MSwgMjAxNy8wNS8wNi0wMTowODoyMSAgICAgICAgIj4gPHJkZjpSREYgeG1sbnM6cmRmPSJodHRwOi8vd3d3LnczLm9yZy8xOTk5LzAyLzIyLXJkZi1zeW50YXgtbnMjIj4gPHJkZjpEZXNjcmlwdGlvbiByZGY6YWJvdXQ9IiIgeG1sbnM6eG1wTU09Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC9tbS8iIHhtbG5zOnN0UmVmPSJodHRwOi8vbnMuYWRvYmUuY29tL3hhcC8xLjAvc1R5cGUvUmVzb3VyY2VSZWYjIiB4bWxuczp4bXA9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC8iIHhtcE1NOk9yaWdpbmFsRG9jdW1lbnRJRD0ieG1wLmRpZDo1ZWI0NWU2Ni01Mzg2LTQ2MGEtOWI4Yi04MGEzYWJmNmU4NzkiIHhtcE1NOkRvY3VtZW50SUQ9InhtcC5kaWQ6QzhFQzdDMjQ1RDFBMTFFODkxMzI5NzRCQzlFMkZENzgiIHhtcE1NOkluc3RhbmNlSUQ9InhtcC5paWQ6QzhFQzdDMjM1RDFBMTFFODkxMzI5NzRCQzlFMkZENzgiIHhtcDpDcmVhdG9yVG9vbD0iQWRvYmUgUGhvdG9zaG9wIENDIDIwMTggKE1hY2ludG9zaCkiPiA8eG1wTU06RGVyaXZlZEZyb20gc3RSZWY6aW5zdGFuY2VJRD0ieG1wLmlpZDpiMmNlN2UxMC1kMjQxLTQxNDQtODc4OS01ZGE2YTdlY2RlZjEiIHN0UmVmOmRvY3VtZW50SUQ9ImFkb2JlOmRvY2lkOnBob3Rvc2hvcDoyZTcwNjUyMS1iNDFmLWI1NGMtOWMz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"
		});
		canvas.resize();

		run();

	}

}


// const canvas = document.createElement('canvas');
// canvas.width = 1000
// canvas.height = 500
// document.body.appendChild(canvas);
// var a = new Bodies(canvas)