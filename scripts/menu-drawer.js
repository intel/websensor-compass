// @license
// Copyright (c) 2018 Intel Corporation. All rights reserved.

// @ts-check
import { LitElement, html } from './lit-element.js';

class MenuElement extends LitElement {
  render() {
    return html`
    <style>
      :host {
        touch-action: none;
        position: fixed;
        top: 0;
        left: 0;
        width: 100%;
        height: 100%;
        overflow: hidden;
      }
      :host::before {
        content: '';
        display: block;
        position: absolute;
        left: 0;
        top: 0;
        width: 100%;
        height: 100%;
        background: rgba(0,0,0,0.4);
        opacity: 0;
        will-change: opacity;
        transition: opacity 0.3s cubic-bezier(0,0,0.3,1);
      }
      .container {
        position: relative;
        width: 90%;
        max-width: 400px;
        background: #FFF;
        height: 100%;
        box-shadow: 2px 0 12px rgba(0,0,0,0.4);
        transform: translateX(-102%);
        display: flex;
        flex-direction: column;
        will-change: transform;
      }
      .animation {
        transition: transform 0.33s cubic-bezier(0,0,0.3,1);
      }
      .expanded {
        transform: none;
      }
      .content {
        margin: 10px;
      }
      button {
        position: fixed;
        padding: 8px;
        margin: 6px 8px 6px 8px;
        color: #039be5;
        background-color: transparent;
        border: 0;
        border-radius: 2px;
        box-shadow: none;
        cursor: pointer;
        display: inline-block;
        height: 40px;
        width: 40px;
        outline: 0;
        transition: background-color .2s, box-shadow .2s;
        vertical-align: middle;
        white-space: nowrap;
      }
    </style>
    <button id="menu-button" on-click=${_ => this.opened = true}>
      <svg fill="#FFFFFF" height="24" viewBox="0 0 24 24" width="24" xmlns="http://www.w3.org/2000/svg">
        <path d="M0 0h24v24H0z" fill="none"/>
        <path d="M3 18h18v-2H3v2zm0-5h18v-2H3v2zm0-7v2h18V6H3z"/>
      </svg>
    </button>
    <div class$="container animation ${this.opened ? 'expanded' : ''}">
      <div class$="content">
        <slot></slot>
      </div>
    </div>`;
  }

  static get properties() {
    return {
      opened: {
        type: Boolean,
        value: false,
        attrName: "opened"
      }
    }
  }

  // Should:
  // - allow being swiped in from the left
  // - on touch end, if majority of menu is visible, animate the rest in, else animate out
  // - allow being swiped out from the right, if open (from the point on it touches)
  // - click outside, closes menu
  // - inertia should close and open depending on direction (so-called fling)
  constructor() {
    super();

    this.onGestureStart = this.onGestureStart.bind(this);
    this.onGestureMove = this.onGestureMove.bind(this);
    this.onTouchEnd = this.onTouchEnd.bind(this);
    this.update = this.update.bind(this);

    this._startX = 0;
    this._currentX = 0;
    this._touching = false;
  }

  async connectedCallback() {
    await super.connectedCallback();

    this.style.visibility = '';

    this.containerEl = this.shadowRoot.querySelector('.container');

    this.addEventListener('pointerdown', this.onGestureStart, { passive: true });
    this.addEventListener('pointermove', this.onGestureMove, { passive: true });
    this.addEventListener('pointerup', this.onTouchEnd);
  }

  isWithinHitRegion(x, y) {
    return x < 20 || this.shadowRoot.elementsFromPoint(x, y).includes(
      this.shadowRoot.querySelector('.container')
    );
  }

  addVelocitySample(dDist, dTime) {
    if (dTime === 0) return;

    const velocitySample = dDist / dTime;

    // Low pass filter.
    const alpha = 0.75;
    this._velocity *= alpha;
    this._velocity += (1 - alpha) * velocitySample;
  }

  onGestureStart(evt) {
    if (!this.isWithinHitRegion(evt.pageX, evt.pageY)) {
      return;
    }

    if (window.PointerEvent) {
      evt.target.setPointerCapture(evt.pointerId);
    }

    this._touching = true;

    this._left = this.containerEl.getBoundingClientRect().left;

    this._startX = evt.pageX;
    this._currentX = this._startX;
    this._timestamp = new Date().getTime();
    this._velocity = 0;

    requestAnimationFrame(this.update);
  }

  onGestureMove(evt) {
    if (!this._touching) {
      return;
    }

    const lastTimestamp = this._timestamp;
    this._timestamp = new Date().getTime();
    const dTime = this._timestamp - lastTimestamp;

    const lastX = this._currentX;
    this._currentX = evt.pageX;

    const dX = this._currentX - lastX;

    this.addVelocitySample(dX, dTime);
  }

  onTouchEnd(evt) {
    if (!this._touching) {
      this.opened = false;
      return;
    }

    this._touching = false;

    let endOpenedState;

    // Check for fling.
    if (Math.abs(this._velocity) > 1) {
      endOpenedState = this._velocity > 0;
    }
    // Check depending on percentage visible.
    else {
      let left = this.containerEl.getBoundingClientRect().left;
      let width = this.containerEl.clientWidth;
      let percentageVisible = (left + width) / width
      endOpenedState = percentageVisible >= 0.5;
    }

    this.containerEl.style.transform = '';
    this.opened = endOpenedState;
  }

  update() {
    if (!this._touching)
      return;

    requestAnimationFrame(this.update);

    const translateX = Math.min(0, this._currentX - this._startX + this._left);
    this.containerEl.style.transform = `translateX(${translateX}px)`;
  }
}

customElements.define('menu-drawer', MenuElement.withProperties());