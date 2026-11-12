/* Virtual joystick using Pointer Events. One instance per thumb. */
class Joystick {
  constructor(root, { onChange, deadZone = 0.08, axis = 'both' } = {}) {
    this.root = root;
    this.base = root.querySelector('.stick-base');
    this.knob = root.querySelector('.stick-knob');
    this.onChange = typeof onChange === 'function' ? onChange : () => {};
    this.deadZone = deadZone;
    // 'both' 允许二维移动；'horizontal' 只允许左右移动，纵向固定为 0。
    this.axis = axis;
    this.pointerId = null;
    this.active = false;
    this.value = { x: 0, y: 0 };
    this.maxTravel = 0;
    this.attach();
  }

  attach() {
    const onDown = (event) => {
      if (this.pointerId !== null) {
        return;
      }
      if (event.pointerType === 'mouse' && event.button !== 0) {
        return;
      }
      this.pointerId = event.pointerId;
      this.active = true;
      this.root.classList.add('active');
      try {
        this.root.setPointerCapture(event.pointerId);
      } catch (_) {
        /* capture is best-effort */
      }
      this.measure();
      this.update(event);
      event.preventDefault();
    };

    const onMove = (event) => {
      if (event.pointerId !== this.pointerId) {
        return;
      }
      this.update(event);
      event.preventDefault();
    };

    const onUp = (event) => {
      if (event.pointerId !== this.pointerId) {
        return;
      }
      this.release();
      event.preventDefault();
    };

    this.root.addEventListener('pointerdown', onDown, { passive: false });
    this.root.addEventListener('pointermove', onMove, { passive: false });
    this.root.addEventListener('pointerup', onUp, { passive: false });
    this.root.addEventListener('pointercancel', onUp, { passive: false });
    this.root.addEventListener('lostpointercapture', onUp, { passive: false });
    this.root.addEventListener('contextmenu', (event) => event.preventDefault());
  }

  measure() {
    const rect = this.base.getBoundingClientRect();
    // Knob diameter is 44% of the base. Travel is base radius minus knob radius.
    this.maxTravel = Math.max(16, rect.width * 0.28);
  }

  update(event) {
    const rect = this.base.getBoundingClientRect();
    const centerX = rect.left + rect.width / 2;
    const centerY = rect.top + rect.height / 2;
    let dx = event.clientX - centerX;
    let dy = event.clientY - centerY;

    if (this.axis === 'horizontal') {
      dy = 0;
    }

    const length = Math.hypot(dx, dy);
    if (length > this.maxTravel) {
      dx = (dx / length) * this.maxTravel;
      dy = (dy / length) * this.maxTravel;
    }

    let nx = dx / this.maxTravel;
    let ny = dy / this.maxTravel;
    const magnitude = Math.hypot(nx, ny);
    if (magnitude < this.deadZone) {
      nx = 0;
      ny = 0;
    } else {
      const scaled = (magnitude - this.deadZone) / (1 - this.deadZone);
      const factor = scaled / magnitude;
      nx *= factor;
      ny *= factor;
    }

    this.value = { x: nx, y: ny };
    this.knob.style.transform = `translate(${dx}px, ${dy}px)`;
    this.onChange(this.value, true);
  }

  release() {
    if (this.pointerId === null && !this.active) {
      return;
    }
    const pointerId = this.pointerId;
    this.pointerId = null;
    this.active = false;
    this.root.classList.remove('active');
    if (pointerId !== null) {
      try {
        this.root.releasePointerCapture(pointerId);
      } catch (_) {
        /* already released */
      }
    }
    this.value = { x: 0, y: 0 };
    this.knob.style.transform = 'translate(0px, 0px)';
    this.onChange(this.value, false);
  }
}

window.Joystick = Joystick;
