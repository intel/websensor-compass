// @license
// Copyright (c) 2018 Intel Corporation. All rights reserved.

// @ts-check
import { LitElement, html } from './lit-element.js';

class ErrorConsole extends LitElement {
  render() {
    return html`
      <style>
        :host {
          pointer-events: none;
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
          pointer-events: all;
          position: absolute;
          bottom: 0px;
          width: 100%;
          max-height: 400px;
          background: #FFF;
          height: 90%;
          box-shadow: 2px 0 12px rgba(0,0,0,0.4);
          transform: translateY(+102%);
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

        .messages {
          flex: 1 1;
          overflow-y: scroll;
          word-wrap: break-word;
          user-select: text;
          transform: translateZ(0);
        }
        .consoleLine {
          padding: 1px 22px 1px 24px;
          border-top: 1px solid rgb(230, 230, 230);
          min-height: 20px;
          line-height: 20px;
          flex: auto;
          font-family: monospace;
          font-size: 8pt;
        }
        .consoleLine:first-of-type {
          border-top: none !important;
        }
        .consoleLine .consoleColor {
          width: 20px;
          height: 20px;
        }
        .error:last-child {
          border-bottom: 1px solid #ffd7d7;
        }
        .log:last-child {
          border-bottom: 1px solid rgb(230, 230, 230);
        }
        .error + :not(.error) {
          border-top: 1px solid #ffd7d7;
        }
        .error {
          background-color: #fff0f0;
          border-top: 1px solid #ffd7d7 !important;
          color: red !important;
        }
        .log {
          background-color: white;
        }

        button {
          pointer-events: all;
          position: fixed;
          bottom: 20px;
          right: 20px;
          padding: 0px;
          margin: 6px 8px 6px 8px;
          color: #039be5;
          background-color: transparent;
          border: 0;
          border-radius: 2px;
          box-shadow: none;
          cursor: pointer;
          height: 48px;
          width: 48px;
          outline: 0;
          transition: background-color .2s, box-shadow .2s;
          vertical-align: middle;
          white-space: nowrap;
        }

        .hidden {
          display: none;
        }
      </style>
      <button id="error-button"
        class$="${ this.entries.length ? '' : 'hidden'}"
        on-click=${_ => this.opened = true }>
        <svg fill="#FFFFFF" height="48" viewBox="0 0 24 24" width="48" xmlns="http://www.w3.org/2000/svg">
          <path d="M0 0h24v24H0z" fill="none"/>
          <path d="M20 8h-2.81c-.45-.78-1.07-1.45-1.82-1.96L17 4.41 15.59 3l-2.17 2.17C12.96 5.06 12.49 5 12 5c-.49 0-.96.06-1.41.17L8.41 3 7 4.41l1.62 1.63C7.88 6.55 7.26 7.22 6.81 8H4v2h2.09c-.05.33-.09.66-.09 1v1H4v2h2v1c0 .34.04.67.09 1H4v2h2.81c1.04 1.79 2.97 3 5.19 3s4.15-1.21 5.19-3H20v-2h-2.09c.05-.33.09-.66.09-1v-1h2v-2h-2v-1c0-.34-.04-.67-.09-1H20V8zm-6 8h-4v-2h4v2zm0-4h-4v-2h4v2z"/>
        </svg>
      </button>
      <div class$="container animation ${this.opened ? 'expanded' : ''}" on-click=${_ => this.opened = !this.opened }>
        <div class$="messages">${this.entries.map(entry =>
          html`<div class$="consoleLine ${entry.type}">${entry.message}.</div>`)}
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

  constructor() {
    super();

    this.entries = [];

    let error = console.error;
    console.error = (message, ...rest) => {
      this.entries.push({ message, type: "error" });
      this.invalidate();

      error.call(console, message, ...rest);
    }

    let log = console.log;
    console.log = (message, ...rest) => {
      this.entries.push({ message, type: "log" });
      this.invalidate();
    
      log.call(console, message, ...rest);
    }
  }
}


customElements.define('error-console', ErrorConsole.withProperties());