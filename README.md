# Test
          return function (e) {
            var x =
              typeof e.data == "string" && e.data.charAt(0) === "{"
                ? JSON.parse(e.data)
                : e.data;
            if (typeof x === "string") {
              if (x == "OK") {
                Pe.worker_initialized = true;
              } else {
                DEBUG_show(x, 2);
                System._browser.console.log(x);
              }
            } else if (Pe.enabled) {
              window.dispatchEvent(new CustomEvent("SA_camera_poseNet_update"));
              ke._needs_RAF = true;
              Pe._bb = null;
              const o = it[Pe.use_holistic ? 1 : 0];
              let e;
              if (Pe.enabled && !o.poseNet) {
                e = true;
                o.poseNet = true;
              }
              if (ge.enabled && !o.handpose) {
                e = true;
                o.handpose = true;
              }
              if (e) DEBUG_show("Pose ML ready", 2);
              Ie =
                (Ee.enabled ? "\n" : "") +
                ke.video_canvas.width +
                "x" +
                ke.video_canvas.height +
                "(" +
                Pe.camera_video_frame_id +
                ")\n";
              let t = ke.video_timestamp;
              let i = 0;
              if (_e) {
                i = Math.max(Math.min(t - _e, 1e3), 10);
                re += i;
                if (++se >= 20) {
                  ae = 1e3 / (re / se);
                  se = re = 0;
                }
              }
              _e = t;
              Pe._t = x._t;
              Be.t_delta = ((Be.t_delta || i) + i) / 2;
              if (x.handpose) {
                i = 0;
                if (ue) {
                  i = Math.max(Math.min(t - ue, 1e3), 10);
                  ce += i;
                  if (++de >= 20) {
                    le = 1e3 / (ce / de);
                    de = ce = 0;
                  }
                }
                ue = t;
                Pe._t_hands = x._t_hands;
                Be.t_delta_hands = ((Be.t_delta_hands || i) + i) / 2;
              }
              if (Pe.use_holistic) {
                o.facemesh = true;
                if (!x.facemesh) {
                  Ee.data_detected = 0;
                }
              }
              let R = [];
              let E = [];
              let T = ke.x_flipped ? 1 : -1;
              let L;
              const s = THREE.MMD.getModels()[0];
              const n = s.mesh.bones_by_name;
              const P = MMD_SA.THREEX.get_model(0).is_T_pose;
              const F = MMD_SA.MMD.motionManager.para_SA;
              const c = F.motion_tracking && F.motion_tracking.arm_as_leg;
              const C = {
                左:
                  c && c.enabled && (!c.linked_side || c.linked_side == "left"),
                右:
                  c &&
                  c.enabled &&
                  (!c.linked_side || c.linked_side == "right"),
              };
              let q, a;
              let K;
              if (Pe.enabled) {
                if (x.posenet && x.posenet.score > 0.3) {
                  Pe.data_detected++;
                  Pe.initial_data_detected = true;
                  if (Pe.data_detected_stable) {
                    if (je) {
                      je = false;
                      s.mesh.visible = true;
                    }
                    if (ke.tilt_adjustment.enabled) {
                      a =
                        (ke.tilt_adjustment.angle *
                          ke.tilt_adjustment.pose_weight *
                          Math.PI) /
                        180;
                      q = new THREE.Quaternion().set(
                        Math.sin(a / 2),
                        0,
                        0,
                        Math.cos(a / 2)
                      );
                    }
                    K = x.posenet;
                    let D = Pe.use_3D_pose;
                    if (Ve) {
                      K._keypoints = K.keypoints;
                      K._keypoints3D = K.keypoints3D;
                      let o = [];
                      let n = [];
                      Ye.forEach((e, t) => {
                        let i = K.keypoints[e];
                        i.part = Ne[t];
                        o.push(i);
                        i = K.keypoints3D[e];
                        i.part = Ne[t];
                        n.push(i);
                      });
                      K.keypoints = o;
                      K.keypoints3D = n;
                    }
                    const d = We ? 0.3 : !Qe ? 0.5 : 0.1;
                    K.keypoints.forEach((e) => {
                      e.score -= d;
                    });
                    if (D) {
                      K._keypoints3D.forEach((e) => {
                        e.score -= d;
                      });
                    }
                    if (Qe) {
                      let i = {};
                      K.keypoints.forEach((e) => {
                        i[e.part] = e;
                      });
                      let o = [];
                      for (let t = 0; t <= 16; t++) {
                        let e = K.keypoints[t];
                        if (!e) o.push(Ze[t]);
                        else if (e.part != Ne[t]) o.push(i[Ne[t]] || Ze[t]);
                        else o.push(e);
                      }
                      K.keypoints = o;
                    }
                    let e = K._keypoints || K.keypoints;
                    let t = e.map((e) => e.position.x);
                    let i = e.map((e) => e.position.y);
                    const h = [
                      Math.min(...i),
                      Math.max(...t),
                      Math.max(...i),
                      Math.min(...t),
                    ];
                    e = K.keypoints.slice(0, 5);
                    t = e.map((e) => e.position.x);
                    i = e.map((e) => e.position.y);
                    const p = [
                      Math.min(...i),
                      Math.max(...t),
                      Math.max(...i),
                      Math.min(...t),
                    ];
                    const f = Math.max(p[1] - p[3], p[2] - p[0]) / 2;
                    h[0] = Math.min(h[0], p[0] - f);
                    h[1] = Math.max(h[1], p[1] + f);
                    h[2] = Math.max(h[2], p[2] - f);
                    h[3] = Math.min(h[3], p[3] - f);
                    Pe._bb = h;
                    if (!Ee.head_pose_enabled) {
                      Ee.bb_center[0] =
                        K.keypoints[0].position.x / ke.video_canvas.width;
                      Ee.bb_center[1] =
                        K.keypoints[0].position.y / ke.video_canvas.height;
                    }
                    if (D) {
                      let e = K._keypoints[0].position;
                      let t = K._keypoints[3].position;
                      let i = K._keypoints[6].position;
                      const y = Te.copy(i);
                      const b = ye.copy(t);
                      b.sub(y);
                      const w = MMD_SA._v3a_.copy(e);
                      const S =
                        (w.x * b.x +
                          w.y * b.y +
                          w.z * b.z -
                          (y.x * b.x + y.y * b.y + y.z * b.z)) /
                        b.lengthSq();
                      const Q = MMD_SA._v3b.set(
                        y.x + b.x * S,
                        y.y + b.y * S,
                        y.z + b.z * S
                      );
                      let o = w.sub(Q).normalize();
                      let n = MMD_SA._v3b_.copy(t).sub(MMD_SA._v3b.copy(i));
                      n.normalize();
                      let a = MMD_SA.TEMP_v3.crossVectors(n, o).normalize();
                      n.crossVectors(o, a);
                      Le.set(
                        n.x,
                        n.y,
                        n.z,
                        0,
                        o.x,
                        o.y,
                        o.z,
                        0,
                        a.x,
                        a.y,
                        a.z,
                        0,
                        0,
                        0,
                        0,
                        1
                      );
                      Se.setFromBasis(Le);
                      let s = Se.conjugate();
                      s = s
                        .clone()
                        .multiply(
                          MMD_SA.TEMP_q.set(
                            Math.sin(Math.PI / 3.5 / 2),
                            0,
                            0,
                            Math.cos(Math.PI / 3.5 / 2)
                          )
                        );
                      Ee._neck.shoulder_center = ye
                        .copy(K._keypoints[11].position)
                        .add(MMD_SA.TEMP_v3.copy(K._keypoints[12].position))
                        .multiplyScalar(0.5)
                        .toArray();
                      if (q) s.premultiply(q);
                      if (!Ee.head_pose_enabled) {
                        Be.add("skin", "首", {
                          after_IK: true,
                          rot: s,
                          onProcessRotation: $e,
                        });
                        Be.skin["首"][1]._rot_mixed &&
                          Be.skin["首"][1].rot.copy(
                            Be.skin["首"][1]._rot_mixed
                          );
                      }
                      Be.add("skin", "首_DUMMY_", {
                        rot: s,
                        _rot_from_pose: s.clone(),
                        onProcessRotation: $e,
                      });
                    }
                    let o = K.keypoints[5];
                    let n = K.keypoints[6];
                    let H = new THREE.Quaternion();
                    L = F.motion_tracking_upper_body_only;
                    Ie +=
                      "\n" +
                      (L
                        ? (F.motion_tracking?.arm_as_leg?.enabled
                            ? "🦶"
                            : "🙋") + "upper body"
                        : "💃full body");
                    const g =
                      MMD_SA_options.Dungeon_options?.item_base.hand_camera;
                    if (g && g._hand_camera_side) {
                      Ie +=
                        "/" +
                        (g.selfie_mode ? "🤳selfie" : "📷handcam") +
                        (g._hand_camera_side == "左" ? "-L" : "-R");
                    }
                    Ie += "\n";
                    if (D && Je) Ce.prepare(K);
                    let M = 0,
                      O = 0;
                    if (ge.enabled && x.handpose)
                      ge.hand_visible_timestamp = {};
                    ze[0] = ze[1] = 0;
                    let k;
                    if (o.score > 0 && n.score > 0) {
                      let m, h;
                      let p, f;
                      if (D) {
                        m = K.keypoints3D[5];
                        h = K.keypoints3D[6];
                        let i = K.keypoints3D[11];
                        let o = K.keypoints3D[12];
                        let n;
                        if (L || i.score <= 0 || o.score <= 0) {
                          L = true;
                        } else {
                          if (
                            K.keypoints[11].position.y >
                              ke.video_canvas.height ||
                            K.keypoints[12].position.y > ke.video_canvas.height
                          ) {
                            L = true;
                          }
                          let e = MMD_SA._v3b.copy(i).sub(o).normalize();
                          let t = MMD_SA.TEMP_v3.set(1, 0, 0);
                          n = be.setFromVectorSpherical(t, e);
                          M = n.z;
                          if (1 || L) n.z = 0;
                          H.setFromEuler(n, "YZX");
                        }
                        let e = MMD_SA.TEMP_v3.set(0, 1, 0);
                        e.y *= -1;
                        let t = MMD_SA._v3a
                          .addVectors(m, h)
                          .multiplyScalar(0.5)
                          .normalize();
                        Ee.calculate_neck_data({
                          is_pose: true,
                          timestamp: Pe.camera_video_frame_id,
                          shoulder_center: Ee._neck.shoulder_center,
                          spine_rot_absolute: ye
                            .setFromVectorSpherical(e, t)
                            .toArray(),
                        });
                        t.applyQuaternion(MMD_SA.TEMP_q.copy(H).conjugate());
                        const W = ye.setFromVectorSpherical(e, t);
                        if (F.motion_tracking_upper_body_only) {
                          W.fromArray(me.filter(W.toArray()));
                          p = new THREE.Quaternion().setFromEuler(W, "XZY");
                        } else {
                          if (L) {
                            if (n) {
                              W.add(n);
                            }
                            H.set(0, 0, 0, 1);
                            W.fromArray(me.filter(W.toArray()));
                            p = new THREE.Quaternion().setFromEuler(W, "YXZ");
                          } else {
                            p = new THREE.Quaternion().setFromEuler(W, "XZY");
                          }
                        }
                        O = W.x;
                        let a = MMD_SA._v3b.copy(m).sub(h).normalize();
                        a.applyQuaternion(
                          MMD_SA.TEMP_q.multiplyQuaternions(H, p).conjugate()
                        );
                        x_axis = MMD_SA.TEMP_v3.set(1, 0, 0);
                        let s = ye.setFromVectorSpherical(x_axis, a);
                        let r = Pe.shoulder_tracking ? 0.5 : 0;
                        let _ =
                          pe.filter(
                            s.z *
                              (MMD_SA_options.user_camera.ML_models.pose
                                .model_quality == "Best"
                                ? 3
                                : 5)
                          ) * r;
                        let l = Math.min(Math.PI / 12 / Math.abs(_), 1);
                        _ *= l;
                        if (L) {
                          s.fromArray(he.filter(s.toArray()));
                        }
                        ze[0] = _;
                        ze[1] = _;
                        s.z *= 1 - r;
                        f = new THREE.Quaternion().setFromEuler(s, "YZX");
                        let d = 0;
                        let c = s.y;
                        let u = (d + c) % (Math.PI * 2);
                        if (u > Math.PI) u = Math.PI * 2 - u;
                        else if (u < 0 && u < -Math.PI) u += Math.PI * 2;
                        p.multiply(
                          ve.set(
                            0,
                            Math.sin((u / 2 - d) / 2),
                            0,
                            Math.cos((u / 2 - d) / 2)
                          )
                        );
                        f.multiply(
                          ve.set(
                            0,
                            Math.sin((u / 2 - c) / 2),
                            0,
                            Math.cos((u / 2 - c) / 2)
                          )
                        );
                        Be.add("skin", "センター", {
                          rot: H.clone(),
                          priority: 9,
                        });
                        if (He.active) {
                          if (
                            !Be.skin["センター"][0].pos &&
                            Be.skin["センター"][1].pos
                          )
                            Be.skin["センター"][0].pos =
                              Be.skin["センター"][1].pos;
                        }
                        const V = p.clone();
                        if (q) {
                          const N = ve
                            .copy(q)
                            .premultiply(MMD_SA.TEMP_q.copy(H).conjugate())
                            .multiply(H);
                          V.premultiply(N);
                        }
                        Be.add("skin", "上半身", { rot: V });
                        Be.add("skin", "上半身2", { rot: f.clone() });
                        k = ve.copy(H).multiply(p).multiply(f);
                        k.conjugate();
                      }
                      const I = MMD_SA.TEMP_v3.set(1, 1, 1 / 3);
                      Pe.shoulder_width = Te.copy(o.position)
                        .multiply(I)
                        .distanceTo(ye.copy(n.position).multiply(I));
                      let A = Math.sqrt(
                        Math.pow(o.position.x - n.position.x, 2) +
                          Math.pow(o.position.y - n.position.y, 2)
                      );
                      let e = 0;
                      if (!D) {
                        e = Math.asin((o.position.y - n.position.y) / A);
                        e =
                          ((Math.max(Math.min(e / (Math.PI / 4), 1), -1) *
                            Math.PI) /
                            4) *
                          T;
                      }
                      let t = T == 1 ? [1, 0] : [0, 1];
                      A = Ee.face_width * 2 || A;
                      Pe._upper_body_only_mode = L;
                      const z = [0, 0];
                      const B = [];
                      t.forEach(function (i, e) {
                        let n, a, s;
                        let r;
                        let _ = e == 0 ? "左" : "右";
                        let l = K.keypoints[5 + i];
                        let d = K.keypoints[7 + i];
                        let c = K.keypoints[9 + i];
                        let u =
                          (MMD_SA_options.user_camera.ML_models.pose
                            .use_armIK == null
                            ? !L &&
                              MMD_SA_options.Dungeon_options?.item_base
                                .hand_camera?._hand_camera_side != _
                            : !MMD_SA_options.user_camera.ML_models.pose
                                .use_armIK) && !C[_];
                        let m = false;
                        let h =
                          c.score > 0 &&
                          c.position.x >= 0 &&
                          c.position.x <= ke.video_canvas.width &&
                          c.position.y >= 0 &&
                          c.position.y <= ke.video_canvas.height;
                        if (F.motion_tracking_upper_body_only && !h) {
                          u = false;
                        }
                        B[e] = new THREE.Quaternion();
                        if (P)
                          B[e].fromArray(
                            MMD_SA.THREEX.utils.convert_A_pose_rotation_to_T_pose(
                              _ + "肩",
                              B[e].toArray()
                            )
                          );
                        let p, f, M;
                        if (D) {
                          p = K.keypoints3D[5 + i];
                          f = K.keypoints3D[7 + i];
                          M = K.keypoints3D[9 + i];
                        }
                        let t = T * (i == 0 ? -1 : 1);
                        let o, g, y;
                        const b = "YZX";
                        if (D && d.score > 0) {
                          o = MMD_SA._v3b
                            .copy(p)
                            .sub(f)
                            .normalize()
                            .applyQuaternion(k);
                          g = MMD_SA.TEMP_v3.set(t, 0, 0);
                          y = new THREE.Quaternion().setFromUnitVectors(g, o);
                          z[e] = ye.setFromVectorSpherical(g, o).z;
                        }
                        let v = fe[_].filters[0].filter;
                        if (c.score > 0) {
                          let e;
                          if (
                            L &&
                            ge.enabled &&
                            ge.use_hands_worker &&
                            !x.handpose
                          ) {
                            if (
                              ge.hand_visible_timestamp[_] + 200 >
                              RAF_timestamp
                            )
                              h = e = true;
                          }
                          let n, t;
                          if (ge.enabled && x.handpose) {
                            n = x.handpose.find(
                              (e) => e.label == (_ == "左" ? "Left" : "Right")
                            );
                            if (n) {
                              h = true;
                            }
                          }
                          if (n) {
                            ge.hand_visible_timestamp[_] = RAF_timestamp;
                            if (!L) ge.hand_visible_session[_] = RAF_timestamp;
                          }
                          if (ge.enabled && L) {
                            let t;
                            let i;
                            let o = ge.hand_visible_session[_] || 0;
                            if (n || e) {
                              if (o < 1e3) {
                                ge.hand_visible_session[_] = 1e3;
                                if (!ge.stabilize_arm_time) {
                                  t = true;
                                } else {
                                  i = true;
                                }
                              } else if (o < 1200) {
                                ge.hand_visible_session[_] += Pe._t_hands;
                                if (
                                  ge.hand_visible_session[_] >
                                  1e3 + ge.stabilize_arm_time
                                ) {
                                  t = true;
                                } else {
                                  i = true;
                                }
                              } else {
                                ge.hand_visible_session[_] = RAF_timestamp;
                              }
                            } else {
                              ge.hand_visible_timestamp[_] = 0;
                              let e = !h;
                              if (h && o < 1e3 + ge.stabilize_arm_time) {
                                if (o >= 1e3)
                                  o = ge.hand_visible_session[_] = 201;
                                e = true;
                                i = true;
                              }
                              if (e) {
                                t = h;
                                if (!o) {
                                  ge.hand_visible_session[_] = 0;
                                  i = true;
                                } else if (o > 200) {
                                  ge.hand_visible_session[_] = 200;
                                } else {
                                  ge.hand_visible_session[_] -= Pe._t_hands;
                                  if (ge.hand_visible_session[_] < 0)
                                    ge.hand_visible_session[_] = 0;
                                }
                              }
                            }
                            if (t) {
                              v.beta = 1 / 10;
                              v = null;
                            }
                            if (
                              ge.stabilize_arm >
                              (F.motion_tracking_upper_body_only ? 0 : 1)
                            ) {
                              if (i) {
                                h = false;
                                n = null;
                                c.score = 0;
                                if (Be.skin[_ + "手首"]?.[0].rot.w != 1) {
                                  const w = new THREE.Quaternion();
                                  Be.add("skin", _ + "ひじ", {
                                    absolute: true,
                                    no_blending: true,
                                    rot: w,
                                  });
                                  Be.add("skin", _ + "手首", {
                                    after_IK: true,
                                    absolute: true,
                                    no_blending: true,
                                    rot: w,
                                  });
                                  Be.add("skin", _ + "手捩", {
                                    after_IK: true,
                                    absolute: true,
                                    no_blending: true,
                                    rot: w,
                                  });
                                }
                              }
                            }
                          }
                          E.push({
                            pos: c.position,
                            dir: i,
                            d: _,
                            visible: h,
                            handpose: n,
                            palm_offset: t,
                          });
                        } else {
                          ge.hand_visible_session[_] = 0;
                        }
                        if (v) {
                          v.beta = 1 / 2;
                        }
                        if (c.score > 0) {
                          if (D) {
                            let o;
                            if (d.score > 0) {
                              let e = Te.copy(f)
                                .sub(M)
                                .normalize()
                                .applyQuaternion(k)
                                .applyQuaternion(
                                  MMD_SA.TEMP_q.copy(y).conjugate()
                                );
                              g = MMD_SA.TEMP_v3.set(t, 0, 0);
                              o = new THREE.Quaternion().setFromUnitVectors(
                                g,
                                e
                              );
                            }
                            if (!u || d.score <= 0) {
                              u = false;
                              let e = MMD_SA._v3a.copy(p).sub(M);
                              let t =
                                d.score > 0
                                  ? MMD_SA.TEMP_v3.copy(p).distanceTo(f) +
                                    MMD_SA.TEMP_v3.copy(f).distanceTo(M)
                                  : 0.5;
                              let i = Math.min(e.length() / t, 1);
                              e.normalize().multiplyScalar(
                                i *
                                  MMD_SA_options.model_para_obj.left_arm_length
                              );
                              n = e.x;
                              a = e.y;
                              s = r = e.z;
                              if (y) {
                                let e;
                                if (C[_]) {
                                  e = k
                                    .clone()
                                    .conjugate()
                                    .multiply(y)
                                    .multiply(o)
                                    .conjugate();
                                }
                                Y(y, _ + "腕");
                                Be.add("skin", _ + "腕", {
                                  absolute: true,
                                  rot: y,
                                  _rot_hand_parent: e,
                                  onFinish: J,
                                });
                                if (o) {
                                  Y(o, _ + "ひじ");
                                  Be.add("skin", _ + "ひじ", {
                                    absolute: true,
                                    rot: o,
                                  });
                                }
                              }
                            } else {
                              Y(o, _ + "ひじ");
                              Y(y, _ + "腕");
                              Be.add("skin", _ + "腕", {
                                absolute: true,
                                rot: y,
                                priority: -1,
                                onFinish: $,
                              });
                              Be.add("skin", _ + "ひじ", {
                                absolute: true,
                                rot: o,
                              });
                            }
                          } else {
                            m = true;
                            let e = l.position.x - c.position.x;
                            let t = l.position.y - c.position.y;
                            n =
                              (e / A) *
                              MMD_SA_options.model_para_obj.shoulder_width *
                              T;
                            a =
                              (t / A) *
                              MMD_SA_options.model_para_obj.shoulder_width;
                            if (d.score > 0) {
                              let e = Math.min(
                                Math.sqrt(
                                  Math.pow(c.position.x - d.position.x, 2) +
                                    Math.pow(c.position.y - d.position.y, 2)
                                ) / A,
                                1
                              );
                              let t = Math.min(
                                Math.sqrt(
                                  Math.pow(l.position.x - d.position.x, 2) +
                                    Math.pow(l.position.y - d.position.y, 2)
                                ) / A,
                                1
                              );
                              r =
                                (0.25 +
                                  ((Math.sin(Math.acos(e)) +
                                    Math.sin(Math.acos(t))) /
                                    2) *
                                    0.75) *
                                MMD_SA_options.model_para_obj.left_arm_length;
                            }
                          }
                        } else {
                          if (C[_]) {
                            Be.add("skin", _ + "手首", {
                              rot: new THREE.Quaternion(),
                            });
                            Be.remove("skin", _ + "足首");
                          }
                          if (d.score > 0) {
                            if (D) {
                              if (!u) {
                                u = false;
                                o = MMD_SA._v3a.copy(p).sub(f).normalize();
                                o.multiplyScalar(
                                  MMD_SA_options.model_para_obj.left_arm_length
                                );
                                n = o.x * T;
                                a = o.y;
                                s = r = o.z;
                              } else {
                                u = true;
                                Y(y, _ + "腕");
                                Be.add("skin", _ + "腕", {
                                  absolute: true,
                                  rot: y,
                                  priority: -1,
                                  onFinish: $,
                                });
                              }
                            } else {
                              u = false;
                              m = true;
                              let e = l.position.x - d.position.x;
                              let t = l.position.y - d.position.y;
                              let i = Math.sqrt(e * e + t * t);
                              let o = Math.asin(e / i);
                              n =
                                Math.sin(o) *
                                MMD_SA_options.model_para_obj.left_arm_length *
                                T;
                              a =
                                Math.cos(o) *
                                MMD_SA_options.model_para_obj.left_arm_length *
                                Math.sign(t);
                            }
                          } else {
                            if (D) {
                              if (!F.motion_tracking_upper_body_only) return;
                              s = r = 0;
                            } else {
                              m = true;
                            }
                            if (n == null) {
                              u = false;
                              n =
                                MMD_SA_options.model_para_obj
                                  .left_arm_to_IK_xy[1] *
                                (_ == "左" ? 1 : -1) *
                                0.2;
                              a = -Math.sqrt(
                                MMD_SA_options.model_para_obj.left_arm_length *
                                  MMD_SA_options.model_para_obj
                                    .left_arm_length -
                                  n * n
                              );
                            }
                          }
                        }
                        if (L && !u) {
                          const S = fe[_].filter([n, a, s]);
                          n = S[0];
                          a = S[1];
                          s = S[2];
                        }
                        R.push({
                          pos: { x: n, y: a, z: s, z_posenet: r },
                          dir: i,
                          d: _,
                          IK_disabled: u,
                          IK_absolute: m,
                        });
                      });
                      z[0] = z[0] == null ? 0 : Math.max(z[0] + Math.PI / 2, 0);
                      z[1] = z[1] == null ? 0 : Math.max(Math.PI / 2 - z[1], 0);
                      const U = z[0] > z[1] ? 1 : 0;
                      const G = Math.min(
                        Math.abs(z[0] - z[1]) / (Math.PI / 2),
                        1
                      );
                      z[U == 0 ? 1 : 0] = 1;
                      z[U] = 1 - G * G * 0.5;
                      t.forEach(function (e, i) {
                        let o = i == 0 ? "左" : "右";
                        if (B[i]) {
                          ze[i] *= z[i];
                          let e = ze[2] * (i == 0 ? 1 : -1);
                          let t = ze[i] + e;
                          const n = Math.PI / 6;
                          if (Math.abs(t) > n) {
                            e = Math.sign(e) * (n - Math.abs(ze[i]));
                            t = ze[i] + e;
                          }
                          t = Ue[o].filter(t);
                          B[i].multiply(
                            MMD_SA.TEMP_q.set(
                              0,
                              0,
                              Math.sin(t / 2),
                              Math.cos(t / 2)
                            )
                          );
                          Be.add("skin", o + "肩", {
                            priority: -2,
                            absolute: !(
                              F.motion_tracking_upper_body_only &&
                              F.motion_tracking?.motion_default_weight?.shoulder
                            ),
                            rot: B[i],
                            _rot_z: ze[i],
                            _rot_shrug: e,
                            _rot_total: t,
                            onFinish: ne,
                          });
                        }
                      });
                    } else {
                      L = true;
                    }
                    if (D && !L) {
                      const X = M;
                      if (q) {
                        H.premultiply(MMD_SA.TEMP_q.copy(q).conjugate());
                        M =
                          X - MMD_SA.TEMP_v3.setEulerFromQuaternion(H, "YZX").z;
                      }
                      Ce.get_hip_center(K);
                      let e = T == 1 ? [1, 0] : [0, 1];
                      let B = [];
                      let t = [];
                      let o = 0;
                      let i = [];
                      e.forEach(function (u, e) {
                        let m = e == 0 ? "左" : "右";
                        let h = K.keypoints3D[11 + u];
                        let p = K.keypoints3D[13 + u];
                        let f = K.keypoints3D[15 + u];
                        let M, g, y;
                        if (f.score > 0) {
                          M = MMD_SA._v3a.copy(h).sub(f);
                          g = M.length();
                          y =
                            p.score > 0
                              ? MMD_SA.TEMP_v3.copy(h).distanceTo(p) +
                                MMD_SA.TEMP_v3.copy(p).distanceTo(f)
                              : 0.7;
                          M.normalize().multiplyScalar(
                            Math.min(g / y, 1) *
                              MMD_SA_options.model_para_obj.left_leg_length
                          );
                        }
                        if (Pe.leg_scale_adjustment && f.score > 0) {
                          let e = 1;
                          let t = 1;
                          let i, o, n;
                          let a, s, r;
                          const _ = be.copy(h);
                          const l = qe.copy(f).sub(_);
                          let d, c;
                          if (p.score > 0) {
                            const e = ye.copy(p);
                            const t =
                              (e.x * l.x +
                                e.y * l.y +
                                e.z * l.z -
                                (_.x * l.x + _.y * l.y + _.z * l.z)) /
                              l.lengthSq();
                            d = Te.set(
                              _.x + l.x * t,
                              _.y + l.y * t,
                              _.z + l.z * t
                            );
                            c = e.sub(d);
                            s = be.copy(h).sub(p).length();
                            r = be.copy(h).sub(d).length();
                            a = c.length();
                          } else {
                            s = y / 2;
                            r = Math.min(l.length() / 2, s);
                            a = Math.sqrt(s * s - r * r);
                          }
                          const z = a / y;
                          const b = ke.video_canvas.width;
                          const v = ke.video_canvas.height;
                          const w = MMD_SA.TEMP_v3;
                          const S = Ce.v_hip;
                          const A = K.keypoints[11 + u];
                          const D = K.keypoints[15 + u];
                          const k =
                            MMD_SA_options.model_para_obj.left_leg_length;
                          const E = b / v;
                          const x = SL.width / SL.height;
                          const P = E < x ? E / x : 1;
                          const I = x < E ? x / E : 1;
                          w.set(
                            ((A.position.x / b) * 2 - 1) * P,
                            (-(A.position.y / v) * 2 + 1) * I,
                            0.5
                          );
                          const R = MMD_SA._v3a_.copy(
                            w
                              .unproject(ke._camera_reset)
                              .sub(ke._camera_reset.position)
                              .normalize()
                          );
                          R.multiplyScalar((S.z - (h.z / y) * k) / R.z);
                          w.set(
                            ((D.position.x / b) * 2 - 1) * P,
                            (-(D.position.y / v) * 2 + 1) * I,
                            0.5
                          );
                          const T = MMD_SA._v3b_.copy(
                            w
                              .unproject(ke._camera_reset)
                              .sub(ke._camera_reset.position)
                              .normalize()
                          );
                          T.multiplyScalar((S.z - (f.z / y) * k) / T.z);
                          g = M.length();
                          e = Math.min(R.distanceTo(T), k) / g;
                          if (p.score > 0) {
                            t =
                              Math.sqrt((s * s - e * e * r * r) / (a * a)) || 0;
                            if (t) {
                            } else {
                              o = 0;
                              n = true;
                              i = s / r;
                            }
                            if (o != null) {
                              Ie +=
                                "\n" +
                                m +
                                ":" +
                                i +
                                "(x" +
                                o +
                                ")" +
                                (n ? "<>" : "") +
                                "\n";
                              e = i;
                              t = o;
                            } else Ie += "\n" + m + ":" + e + "(x" + t + ")\n";
                            if (e != 1) {
                              const L = be
                                .copy(h)
                                .add(l.multiplyScalar(e))
                                .sub(f);
                              for (const F of [27, 29, 31]) {
                                const e = K._keypoints3D[F + u];
                                Object.assign(e, MMD_SA.TEMP_v3.copy(e).add(L));
                              }
                              f = K.keypoints3D[15 + u];
                              const j = d
                                .sub(h)
                                .multiplyScalar(e)
                                .add(h)
                                .add(c.multiplyScalar(t));
                              p = K.keypoints3D[13 + u] = Object.assign(p, j);
                              M.multiplyScalar(e);
                            }
                          } else {
                            M.multiplyScalar(e);
                            Ie += "\n" + m + "(IK):" + e + "(x" + t + ")\n";
                          }
                        }
                        let n, a;
                        let i;
                        if (p.score > 0) {
                          i = MMD_SA._v3b
                            .copy(h)
                            .sub(p)
                            .normalize()
                            .applyQuaternion(MMD_SA.TEMP_q.copy(H).conjugate());
                          let e = MMD_SA.TEMP_v3.set(0, 1, 0);
                          e.y *= -1;
                          a = ye.setFromVectorSpherical(e, i);
                          n = new THREE.Quaternion().setFromEuler(a, "XZY");
                          const t = Te.setFromVectorSpherical(
                            e,
                            MMD_SA._v3b
                              .copy(h)
                              .sub(p)
                              .normalize()
                              .applyQuaternion(
                                MMD_SA.TEMP_q.copy(H)
                                  .multiply(
                                    MMD_SA._q2.set(
                                      0,
                                      0,
                                      Math.sin(X / 2),
                                      Math.cos(X / 2)
                                    )
                                  )
                                  .multiply(
                                    MMD_SA._q1.set(
                                      Math.sin(O / 2),
                                      0,
                                      0,
                                      Math.cos(O / 2)
                                    )
                                  )
                                  .conjugate()
                              )
                          );
                          o += t.x;
                        }
                        let t =
                          !MMD_SA_options.user_camera.ML_models.pose.use_legIK;
                        if (f.score > 0) {
                          let i, o;
                          if (p.score > 0) {
                            i = Te.copy(p)
                              .sub(f)
                              .normalize()
                              .applyQuaternion(
                                MMD_SA.TEMP_q.copy(H).conjugate()
                              );
                            o = be
                              .copy(i)
                              .applyQuaternion(
                                MMD_SA.TEMP_q.copy(n).conjugate()
                              );
                            o.z = Math.max(-o.z, 0);
                            o.x *= -1;
                            let e = o.normalize().toSphericalCoords();
                            let t = Math.sqrt(
                              Math.min((Math.PI - e[2]) / (Math.PI / 2), 1)
                            );
                            a.y = e[1] * t;
                            n.setFromEuler(a, "XZY");
                          }
                          if (!t || p.score <= 0) {
                            Pe.enable_IK(m + "足ＩＫ", true);
                            B.push({ pos: M.clone(), rot: [n], dir: u, d: m });
                          } else {
                            o = be
                              .copy(i)
                              .applyQuaternion(
                                MMD_SA.TEMP_q.copy(n).conjugate()
                              );
                            let e = MMD_SA.TEMP_v3.set(0, 1, 0);
                            e.y *= -1;
                            let t =
                              new THREE.Quaternion().setFromVectorSpherical(
                                e,
                                o
                              );
                            Pe.enable_IK(m + "足ＩＫ", false);
                            B.push({ rot: [n, t], dir: u, d: m });
                          }
                          const s = K._keypoints3D[29 + u];
                          const r = K._keypoints3D[31 + u];
                          if (s.score > 0 && r.score > 0) {
                            let e = MMD_SA._v3a_.copy(s).sub(f).normalize();
                            let t = MMD_SA._v3b_.copy(s).sub(r).normalize();
                            let i = Te.crossVectors(e, t).normalize();
                            e.crossVectors(t, i);
                            Le.set(
                              i.x,
                              i.y,
                              i.z,
                              0,
                              e.x,
                              e.y,
                              e.z,
                              0,
                              t.x,
                              t.y,
                              t.z,
                              0,
                              0,
                              0,
                              0,
                              1
                            );
                            Se.setFromBasis(Le);
                            let o = Se.conjugate();
                            const _ =
                              -Math.PI /
                              (MMD_SA_options.user_camera.ML_models.pose
                                .model_quality == "Best"
                                ? 10
                                : 8);
                            o = o
                              .clone()
                              .multiply(
                                MMD_SA.TEMP_q.set(
                                  Math.sin(_ / 2),
                                  0,
                                  0,
                                  Math.cos(_ / 2)
                                )
                              );
                            if (q) o.premultiply(q);
                            const l = ve.copy(H);
                            if (q) l.premultiply(q);
                            const d = MMD_SA.TEMP_v3.setEulerFromQuaternion(
                              l,
                              "YZX"
                            );
                            const c = Math.abs(d.y) % Math.PI;
                            let n =
                              c > Math.PI / 2
                                ? 1 - (c - Math.PI / 2) / (Math.PI / 2)
                                : 1;
                            if (
                              MMD_SA_options.user_camera.ML_models.pose
                                .use_legIK
                            ) {
                              const C = o.clone();
                              if (n < 1) {
                                d.x = d.z = 0;
                                C.slerp(we.setFromEuler(d, "YZX"), 1 - n);
                              }
                              B[B.length - 1].rot[2] = C;
                            }
                            Be.add("skin", m + "足首", {
                              after_IK: true,
                              absolute: true,
                              parent_based: true,
                              motion_recorder_disabled:
                                MMD_SA_options.user_camera.ML_models.pose
                                  .use_legIK,
                              rot: o,
                              ratio: n,
                              ratio_euler:
                                MMD_SA_options.user_camera.ML_models.pose
                                  .model_quality == "Best"
                                  ? { x: 1, y: 1, z: 0.5, order: "YXZ" }
                                  : null,
                            });
                          }
                        } else {
                          if (p.score > 0) {
                            if (!t) {
                              M = MMD_SA._v3a
                                .copy(h)
                                .sub(p)
                                .normalize()
                                .multiplyScalar(
                                  MMD_SA_options.model_para_obj.left_leg_length
                                );
                              if (q) M.applyQuaternion(q);
                              Pe.enable_IK(m + "足ＩＫ", true);
                              B.push({
                                pos: M.clone(),
                                rot: [n],
                                dir: u,
                                d: m,
                              });
                            } else {
                              Pe.enable_IK(m + "足ＩＫ", false);
                              B.push({ rot: [n], dir: u, d: m });
                            }
                          }
                          if (Be.skin[m + "足首"]?.[0].rot.w != 1)
                            Be.add("skin", m + "足首", {
                              after_IK: true,
                              absolute: true,
                              no_blending: true,
                              rot: new THREE.Quaternion(),
                            });
                        }
                      });
                      let n = 0;
                      i.forEach((e) => {
                        const t = e.y * (1 - e.scale);
                        if (t > 0) {
                          n = Math.max(t, n);
                        } else if (i.length > 1) {
                          n = Math.max(t, n || t);
                        }
                      });
                      if (n) {
                        Ce.v_hip.y += n;
                        Ie += "\nleg_offset_y:" + n + "\n";
                      }
                      o *= 0.5 * 0.5;
                      o += O;
                      let a = new THREE.Quaternion().setFromEuler(
                        MMD_SA.TEMP_v3.set(o, 0, M),
                        "YZX"
                      );
                      Be.add("skin", "下半身", {
                        absolute: true,
                        rot: a.clone(),
                      });
                      if (!L) {
                        Be.add("skin", "全ての親", {
                          pos: new THREE.Vector3(),
                          after_IK: true,
                          priority: 999,
                          onProcessPosition: ee,
                        });
                      }
                      a.conjugate();
                      B.forEach((e) => {
                        var t = e.d;
                        if (e.rot[0]) {
                          e.rot[0].multiplyQuaternions(a, e.rot[0]);
                          Be.add("skin", t + "足", {
                            absolute: true,
                            rot: e.rot[0],
                          });
                        }
                        if (e.pos) {
                          e.pos.add(
                            MMD_SA_options.model_para_obj.leg_IK_offset[t]
                          );
                          const i =
                            e.rot[2] ||
                            (Be.skin[t + "足ＩＫ"] &&
                              Be.skin[t + "足ＩＫ"][0].rot);
                          Be.add("skin", t + "足ＩＫ", {
                            absolute: true,
                            pos: e.pos,
                            rot: i,
                            priority: 999,
                            onFinish: Z,
                          });
                        } else {
                          if (e.rot[1]) {
                            Be.add("skin", t + "ひざ", {
                              absolute: true,
                              rot: e.rot[1],
                            });
                          } else if (Be.skin[t + "ひざ"]?.[0].rot.w != 1) {
                            Be.add("skin", t + "ひざ", {
                              absolute: true,
                              no_blending: true,
                              rot: new THREE.Quaternion(),
                            });
                          }
                        }
                      });
                    } else {
                      if (F.motion_tracking_upper_body_only) {
                        Be.add("skin", "全ての親", {
                          is_dummy: true,
                          pos: true,
                          after_IK: true,
                          priority: 999,
                        });
                      } else if (
                        D &&
                        o.score > 0 &&
                        n.score > 0 &&
                        ke.video_canvas.width
                      ) {
                        Ce.get_hip_center(K);
                        Be.add("skin", "全ての親", {
                          pos: new THREE.Vector3(),
                          after_IK: true,
                          priority: 999,
                          onProcessPosition: ee,
                        });
                      }
                      if (!C["左"] || !C["右"])
                        Be.add("skin", "下半身", { is_dummy: true, rot: true });
                      for (const A of ["左", "右"]) {
                        if (!C[A]) {
                          Be.add("skin", A + "足ＩＫ", {
                            is_dummy: true,
                            pos: true,
                            priority: 999,
                          });
                          Be.add("skin", A + "足", {
                            is_dummy: true,
                            rot: true,
                          });
                          Be.add("skin", A + "ひざ", {
                            is_dummy: true,
                            rot: true,
                          });
                          Be.add("skin", A + "足首", {
                            is_dummy: true,
                            after_IK: true,
                            rot: true,
                          });
                        }
                      }
                    }
                  }
                } else {
                  Pe.data_detected = 0;
                }
              }
              if (Pe.enabled && !Pe.data_detected_stable && !je) {
                je = true;
                if (
                  !F.motion_tracking_upper_body_only &&
                  Pe.initial_data_detected
                )
                  s.mesh.visible = false;
              }
              let j;
              let v = [];
              const r = !ge.enabled && K && Pe.use_3D_pose;
              const _ = T == 1 ? [1, 0] : [0, 1];
              const u = {};
              const l =
                ge.enabled && Pe.shoulder_width
                  ? Math.min(
                      Math.max(
                        Math.max(
                          ke.video_canvas.width,
                          ke.video_canvas.height
                        ) /
                          Pe.shoulder_width -
                          5,
                        0
                      ) / 5,
                      1
                    ) * 0.5
                  : 0;
              if (r || (!F.motion_tracking_upper_body_only && l)) {
                _.forEach(function (e, t) {
                  let i = t == 0 ? "左" : "右";
                  if (!E.find((e) => e.d == i)?.visible) return;
                  let o = K._keypoints3D[15 + e];
                  let n = K._keypoints3D[17 + e];
                  let a = K._keypoints3D[19 + e];
                  if (o.score <= 0 || n.score <= 0 || a.score <= 0) return;
                  let s = e == 1 ? [a, n] : [n, a];
                  let r = MMD_SA._v3a_
                    .copy(o)
                    .sub(MMD_SA._v3b.copy(n).add(a).multiplyScalar(0.5))
                    .normalize();
                  r.x = r.x * T;
                  let _ = MMD_SA._v3b_.copy(s[0]).sub(s[1]).normalize();
                  _.z = _.z * T;
                  _.y = _.y * T;
                  let l = MMD_SA.TEMP_v3.crossVectors(_, r).normalize();
                  _.crossVectors(r, l);
                  Le.set(
                    _.x,
                    _.y,
                    _.z,
                    0,
                    r.x,
                    r.y,
                    r.z,
                    0,
                    l.x,
                    l.y,
                    l.z,
                    0,
                    0,
                    0,
                    0,
                    1
                  );
                  Se.setFromBasis(Le);
                  let d = new THREE.Quaternion();
                  d.copy(Se).conjugate();
                  if (q) d.premultiply(q);
                  u[i] = d;
                });
              }
              if (ge.enabled && x.handpose && x.handpose.length && E.length) {
                const M = Xe ? 1 : 2;
                x.handpose.forEach((e) => {
                  e._offset = {};
                  j = e.annotations;
                  if (M > 1) {
                    for (let e in j) {
                      j[e].forEach((e) => {
                        e[2] *= M;
                      });
                    }
                  }
                  let t = MMD_SA.TEMP_v3.fromArray(j.palm[0]);
                });
                E.forEach((u, e) => {
                  var m = u.dir;
                  let h = u.handpose;
                  if (h && u.visible) {
                    u.visible = true;
                    h._used = true;
                    let k = u.d;
                    h._d = k;
                    j = h.annotations;
                    let e = R.find((e) => e.dir == m);
                    let t = Math.max(
                      (4 +
                        (MMD_SA_options.user_camera.ML_models.hands
                          .depth_adjustment || 0)) /
                        8,
                      0
                    );
                    if (
                      t &&
                      L &&
                      e.pos.z > 0 &&
                      MMD_SA_options.Dungeon_options?.item_base.hand_camera
                        ?._hand_camera_side != k
                    ) {
                      const M =
                        (MMD_SA._v3a
                          .fromArray(j.palm[0])
                          .distanceTo(MMD_SA._v3b.fromArray(j.middle[0])) +
                          MMD_SA._v3a
                            .fromArray(j.index[0])
                            .distanceTo(MMD_SA._v3b.fromArray(j.pinky[0]))) /
                        2 /
                        Pe.shoulder_width;
                      const g =
                        (0.25 *
                          (10 +
                            (MMD_SA_options.user_camera.ML_models.hands
                              .palm_shoulder_scale || 0))) /
                        10;
                      const y =
                        Math.max(
                          ke.video_canvas.width,
                          ke.video_canvas.height
                        ) / Pe.shoulder_width;
                      palm_far_scale =
                        ((1.8 + (1 - Math.min(Math.max(y - 4, 0), 1)) * 0.4) *
                          (8 -
                            (MMD_SA_options.user_camera.ML_models.hands
                              .depth_scale || 0))) /
                        8;
                      Me[k].filter(
                        Math.min(
                          Math.max(M - g, 0) / (g * palm_far_scale - g),
                          1
                        )
                      );
                    }
                    v.push(k + "手");
                    let i =
                      m == 1
                        ? [j.index[0], j.ring[0]]
                        : [j.ring[0], j.index[0]];
                    let o, n, a;
                    let s = MMD_SA._v3a_
                      .fromArray(j.palm[0])
                      .sub(MMD_SA._v3b.fromArray(j.middle[0]))
                      .normalize();
                    s.x = s.x * T;
                    let r = MMD_SA._v3b_
                      .fromArray(i[0])
                      .sub(MMD_SA._v3b.fromArray(i[1]))
                      .normalize();
                    r.z = r.z * T;
                    r.y = r.y * T;
                    let _ = MMD_SA.TEMP_v3.crossVectors(r, s).normalize();
                    r.crossVectors(s, _);
                    Le.set(
                      r.x,
                      r.y,
                      r.z,
                      0,
                      s.x,
                      s.y,
                      s.z,
                      0,
                      _.x,
                      _.y,
                      _.z,
                      0,
                      0,
                      0,
                      0,
                      1
                    );
                    Se.setFromBasis(Le);
                    let l = new THREE.Quaternion();
                    l.copy(Se).conjugate();
                    if (ke.tilt_adjustment.enabled) {
                      const b = (ke.tilt_adjustment.angle * Math.PI) / 180;
                      l.premultiply(
                        MMD_SA.TEMP_q.set(
                          Math.sin(b / 2),
                          0,
                          0,
                          Math.cos(b / 2)
                        )
                      );
                    }
                    Be.add("skin", k + "手首", {
                      after_IK: true,
                      rot: l,
                      onProcessRotation: oe,
                    });
                    Be.add("skin", k + "手捩", {
                      after_IK: true,
                      absolute: true,
                      priority: 1,
                      no_blending: true,
                      rot: new THREE.Quaternion(),
                    });
                    const p = MMD_SA.TEMP_v3.setEulerFromQuaternion(l, "YXZ").y;
                    const f = 1;
                    const P =
                      (Math.abs(p) < Math.PI / 6
                        ? 1
                        : Math.abs(Math.abs(p) - Math.PI) < Math.PI / 6
                        ? -1
                        : 0) * f;
                    const I =
                      Math.max((Math.abs(p) - Math.PI / 2) * f, 0) /
                      (Math.PI / 2);
                    let d = new THREE.Quaternion();
                    let c = MMD_SA_options.model_para_obj.finger_base;
                    let E = ve.copy(Se);
                    E.x *= -1;
                    E.z *= -1;
                    let x = k == "左" ? 1 : -1;
                    Ae.forEach((h, p) => {
                      let f = c[(T == 1 ? k : k == "左" ? "右" : "左") + p];
                      let M = f.base_index + (p == 0 ? 0 : -1);
                      let g = j[Ge[p]];
                      let y = we.copy(E);
                      const b = "XZY";
                      if (P && p > 0) {
                        const e = Math.max(
                          MMD_SA._v3a
                            .fromArray(g[2])
                            .sub(MMD_SA._v3b.fromArray(g[1]))
                            .length() * 1.2,
                          MMD_SA._v3a
                            .fromArray(j.palm[0])
                            .sub(MMD_SA._v3b.fromArray(g[0]))
                            .length() * (P == 1 ? 0.2 : 0.3)
                        );
                        const S = MMD_SA._v3a
                          .fromArray(g[1])
                          .sub(MMD_SA._v3b.fromArray(g[0]));
                        if (e / S.length() > 1) {
                          const t =
                            -P * Math.sqrt(e * e - (S.x * S.x + S.y * S.y)) -
                            S.z;
                          for (let e = 1; e < 3; e++) g[e][2] += t;
                        }
                      }
                      const v = k + h + "指" + De[f.base_index];
                      const w = p == 0 ? 1 : 0;
                      for (let m = M; m < 3; m++) {
                        let e = k + h + "指" + De[f.base_index + (m - M)];
                        let t = MMD_SA._v3a.fromArray(g[m + 0]);
                        let i = MMD_SA._v3b.fromArray(g[m + 1]);
                        let o = MMD_SA.TEMP_v3.fromArray(
                          m == M ? j.palm[0] : g[m - 1]
                        );
                        t.y = -t.y;
                        i.y = -i.y;
                        o.y = -o.y;
                        let n = MMD_SA._v3a_.copy(t).sub(o);
                        let a = MMD_SA._v3b_.copy(i).sub(t);
                        n.normalize();
                        a.normalize();
                        let s, r;
                        let _;
                        n.applyQuaternion(y);
                        a.applyQuaternion(y);
                        if (m == M) {
                          let e = MMD_SA.TEMP_q.set(0, 0, 0, 1);
                          e.setFromVectorSpherical(
                            MMD_SA.TEMP_v3.set(0, 1, 0),
                            n
                          ).conjugate();
                          a.applyQuaternion(e);
                          y.multiplyQuaternions(e, y);
                        }
                        let l = MMD_SA._q1.set(0, 0, 0, 1);
                        let d = MMD_SA._q2.set(0, 0, 0, 1);
                        n.set(0, 1, 0);
                        _ = ye.setFromVectorSpherical(n, a);
                        l.setFromEuler(_, b);
                        d.copy(l);
                        if (
                          Math.abs(_.z) > Math.PI / (p == 0 ? 1.1 : 2.5) ||
                          _.x > Math.PI / (p == 0 ? 1.1 : 3)
                        ) {
                          _.set(-Math.abs(n.angleTo(a)), 0, 0);
                        }
                        if (p == 0 && m > M + 1) {
                          _.x = 0;
                        } else if (_.x > 0) {
                          _.x *= p == 0 ? 0 : m > M ? 0 : 0.75;
                        } else {
                          _.x = Math.max(_.x, -Math.PI / (p == 0 ? 1.25 : 2));
                        }
                        if (p == 0 || m == M) {
                          _.z +=
                            ((((p - 2) / 2) * Math.PI) / 8) *
                            x *
                            (p > 0 ? 1 : 0.5);
                          _.z *=
                            p > 0
                              ? Math.min(
                                  Math.max(Math.PI / 2 - Math.abs(_.x), 0) /
                                    (Math.PI / 2.5),
                                  1
                                )
                              : 1;
                        } else {
                          _.z = 0;
                        }
                        if (I && p > 0 && m == M) {
                          const S = MMD_SA._v3a.fromArray(g[m + 1]);
                          S.y *= -1;
                          S.applyQuaternion(E);
                          const D = MMD_SA._v3b.fromArray(g[m + 3]);
                          D.y *= -1;
                          D.applyQuaternion(E);
                          if (S.y > D.y) {
                            _.x -= Math.abs(_.z) * I;
                            _.z *= 1 - I;
                          }
                        }
                        _.y = 0;
                        if (p == 0 && m == w) _.multiplyScalar(1.5);
                        if (p == 0) y.multiplyQuaternions(l.conjugate(), y);
                        l.setFromEuler(_, b);
                        if (p > 0) {
                          y.multiplyQuaternions(l.conjugate(), y);
                          l.conjugate();
                        }
                        let c = e;
                        let u = l.toAxisAngle();
                        r = u[0];
                        s = u[1];
                        r.z *= -1;
                        const A =
                          !MMD_SA.THREEX.enabled && p == 0
                            ? Math.min(
                                (Math.PI / 8 -
                                  Fe[v]._axis_rot_offset_inv_z * x) /
                                  (Math.PI / 8),
                                1.5
                              )
                            : 1;
                        if (!MMD_SA.THREEX.enabled && p == 0)
                          r.applyEuler(
                            MMD_SA.TEMP_v3.set(0, (Math.PI / 4) * x * A, 0)
                          );
                        r.applyEuler(
                          Te.set(
                            MMD_SA.THREEX.enabled ? Math.PI / 2 : 0,
                            (-Math.PI / 2) * x,
                            0
                          ),
                          "YXZ"
                        );
                        if (MMD_SA.THREEX.enabled) {
                          l.setFromAxisAngle(r, s);
                        } else {
                          r.applyQuaternion(Fe[c].axis_rot);
                          l.setFromAxisAngle(r, s);
                        }
                        if (m == w)
                          l.premultiply(
                            MMD_SA.TEMP_q.set(0, 0, 0, 1).slerp(
                              Fe[v].axis_rot_offset_inv,
                              p == 0 ? 0.5 * A : 1
                            )
                          );
                        if (C[k]) {
                          if (m == M) {
                            const e = Be.skin[k + "手首"][0];
                            if (!e._finger_x) e._finger_x = [];
                            e._finger_x[p] = _.x;
                          }
                        } else {
                          Be.add("skin", e, { absolute: true, rot: l.clone() });
                        }
                      }
                    });
                  } else {
                    const e = u.d;
                    if (
                      F.motion_tracking_upper_body_only &&
                      ge.stabilize_arm &&
                      Be.get_blend_default_motion(
                        "skin",
                        e + (C[e] ? "足首" : "手首")
                      ) != 0
                    ) {
                      u.visible = false;
                    }
                  }
                });
              } else if (r) {
                _.forEach(function (e, t) {
                  let i = t == 0 ? "左" : "右";
                  if (u[i]) {
                    Be.add("skin", i + "手首", {
                      after_IK: true,
                      rot: u[i],
                      onProcessRotation: oe,
                    });
                    Be.add("skin", i + "手捩", {
                      after_IK: true,
                      absolute: true,
                      priority: 1,
                      no_blending: true,
                      rot: new THREE.Quaternion(),
                    });
                  }
                });
              }
              for (const A of ["左", "右"]) {
                const D = Be.skin[A + "手首"];
                if (D) {
                  D[0]._rot_pose = u[A];
                  D[0]._rot_pose_ratio = l;
                  if (!D[0]._finger_x) D[0]._finger_x = D[1]._finger_x;
                }
              }
              const m = [];
              R.forEach(function (o) {
                var n = o.d;
                let a = Math.max(
                  (4 +
                    (MMD_SA_options.user_camera.ML_models.hands
                      .depth_adjustment || 0)) /
                    8,
                  0
                );
                if (
                  a &&
                  ge.enabled &&
                  L &&
                  MMD_SA_options.Dungeon_options?.item_base.hand_camera
                    ?._hand_camera_side != n
                ) {
                  let t = Me[n].filter();
                  let e = (RAF_timestamp - Me[n].timestamp) / 1e3;
                  let i = 1 - Math.min(Math.max(e - 0.2, 0) / (1 - 0.2), 1);
                  if (i) {
                    a *= i;
                    let e =
                      t * MMD_SA_options.model_para_obj.left_arm_length * a +
                      o.pos.z * (1 - a);
                    const s = Math.sqrt(
                      MMD_SA_options.model_para_obj.left_arm_length *
                        MMD_SA_options.model_para_obj.left_arm_length -
                        o.pos.x * o.pos.x -
                        o.pos.y * o.pos.y
                    );
                    e = Math.min(e, s);
                    o.pos.z = e;
                  }
                }
                if (C[n]) {
                  if (o.IK_disabled) return;
                  Pe.enable_IK(n + "足ＩＫ", true);
                  if (
                    Be.skin[n + "手首"] &&
                    Be.skin[n + "腕"] &&
                    Be.skin[n + "手首"][0].timestamp ==
                      Be.skin[n + "腕"][0].timestamp
                  ) {
                    let e;
                    if (e) {
                    } else {
                      const r = MMD_SA.TEMP_q.setFromEuler(
                        MMD_SA.TEMP_v3.set(Math.PI / 2, 0, 0)
                      );
                      const _ = Be.skin[n + "手首"][0].rot
                        .premultiply(r)
                        .multiply(r.conjugate());
                      Be.add("skin", n + "足首", {
                        after_IK: true,
                        rot: _,
                        _finger_x: Be.skin[n + "手首"][0]._finger_x,
                        absolute: true,
                        onFinish: ie,
                      });
                    }
                  } else {
                    const l = Be.skin[n + "足首"];
                    if (l) {
                      l[0].rot_parent = l[0]._rot_parent;
                    }
                  }
                  Be.remove("skin", n + "手首");
                  Be.remove("skin", n + "腕");
                  Be.remove("skin", n + "ひじ");
                  Be.remove("skin", n + "手捩");
                  Be.remove("skin", n + "腕ＩＫ");
                  if (ge.enabled) {
                    Ae.forEach((t, e) => {
                      let i = e == 0 ? 0 : 1;
                      for (let e = i; e < i + 3; e++)
                        Be.remove("skin", n + t + "指" + De[e]);
                    });
                  }
                  const t = new THREE.Vector3().copy(o.pos);
                  const e = c.transformation?.position;
                  const i = O(n + "足ＩＫ", t, e, (e) => {
                    const t = MMD_SA_options.model_para_obj.left_leg_length;
                    e.y += (t * 1) / 3;
                    e.z += t * 0.25;
                    e.multiplyScalar(1.5);
                  });
                  m.push({ d: n, pos: t, rot: i[1] });
                } else {
                  Pe.enable_IK(n + "腕ＩＫ", !o.IK_disabled);
                  if (!o.IK_disabled) {
                    if (o.pos.z == null)
                      o.pos.z =
                        o.pos.z_posenet ||
                        (Pe.use_3D_pose
                          ? 0
                          : MMD_SA_options.model_para_obj.left_arm_length *
                            0.2);
                    const d = new THREE.Vector3().copy(o.pos);
                    Be.add("skin", n + "腕ＩＫ", {
                      priority: 999,
                      pos: d,
                      _IK_absolute: o.IK_absolute,
                      onProcessPosition: te,
                    });
                  }
                  const e = Be.skin[n + "手首"];
                  if (e) {
                    e[0].rot_parent = e[0]._rot_parent;
                  }
                }
              });
              if (m.length && c.transformation?.position?.process)
                c.transformation.position.process(m);
              m.forEach((e) => {
                const t = e.d;
                const i = e.pos;
                i.y +=
                  MMD_SA_options.model_para_obj.left_leg_IK[1] +
                  (n["センター"].position.y - n["センター"].pmxBone.origin[1]);
                if (e.rot) {
                  Be.add("skin", t + "足", {
                    absolute: true,
                    rot: e.rot,
                    onFinish: function (e, t) {
                      this.skin[t][0]._rot_ =
                        e.bones_by_name[t].quaternion.clone();
                    },
                  });
                } else {
                  Be.remove("skin", t + "足");
                }
                Be.remove("skin", t + "ひざ");
                Be.add("skin", t + "足ＩＫ", {
                  absolute: true,
                  priority: 999,
                  pos: i,
                });
              });
              if (je) {
                if (F.motion_tracking_upper_body_only) {
                  for (const A of ["左", "右"]) {
                    Pe.enable_IK(A + "腕ＩＫ", F.has_arm_IK);
                  }
                }
              } else if (F.motion_tracking_upper_body_only) {
                for (const A of ["左", "右"]) {
                  const k = !E.find((e) => e.d == A)?.visible;
                  if (C[A]) {
                    Be.set_blend_default_motion("skin", A + "足ＩＫ", k);
                    Be.set_blend_default_motion("skin", A + "足", k);
                    Be.set_blend_default_motion("skin", A + "ひざ", k);
                    Be.set_blend_default_motion("skin", A + "足首", k);
                  } else {
                    if (F.has_leg_IK) {
                      Be.set_blend_default_motion("skin", A + "足ＩＫ", true);
                      Be.remove("skin", A + "足首");
                    } else {
                      Be.set_blend_default_motion("skin", A + "足", true);
                      Be.set_blend_default_motion("skin", A + "ひざ", true);
                      Be.set_blend_default_motion("skin", A + "足首", true);
                    }
                    Pe.enable_IK(A + "足ＩＫ", F.has_leg_IK);
                    Pe.enable_IK(A + "つま先ＩＫ", F.has_leg_IK);
                    Be.set_blend_default_motion("skin", A + "肩", k);
                    Be.set_blend_default_motion("skin", A + "腕", k);
                    Be.set_blend_default_motion("skin", A + "ひじ", k);
                    Be.set_blend_default_motion("skin", A + "手捩", k);
                    Be.set_blend_default_motion("skin", A + "手首", k);
                    Be.set_blend_default_motion("skin", A + "腕ＩＫ", k);
                    if (ge.enabled) {
                      Ae.forEach((t, e) => {
                        let i = e == 0 ? 0 : 1;
                        for (let e = i; e < i + 3; e++)
                          Be.set_blend_default_motion(
                            "skin",
                            A + t + "指" + De[e],
                            k
                          );
                      });
                    }
                  }
                }
              }
              Ie += System._browser.motion_control.debug_msg;
              Ie +=
                (Ie ? "" : "\n") +
                "P-FPS:" +
                Math.round(ae) +
                "/" +
                Math.round(x.fps || 0) +
                (ge.use_hands_worker ? "\nH-FPS:" + Math.round(le) : "");
              System._browser.motion_control.process({ posenet_data: x });
              if (ke.motion_recorder.speed) {
                const H = ke.motion_recorder.stats;
                Ie +=
                  "\n🔴 " +
                  ke.motion_recorder.time +
                  "(-" +
                  H[0] +
                  "/" +
                  H[1] +
                  "-" +
                  H[3] +
                  "/" +
                  H[2] +
                  "-" +
                  H[4] +
                  ")";
              }
              if (
                Ie &&
                !System._browser.overlay_mode &&
                !MMD_SA_options.user_camera.ML_models.debug_hidden &&
                (Pe.use_holistic ? !x.facemesh : !Ee.enabled)
              ) {
                System._browser.on_animation_update.add(
                  () => {
                    DEBUG_show(
                      ((Pe.use_holistic && "(no facemesh data)\n") || "") +
                        (Re ? "\n" + Re + "\n" : "") +
                        Ie +
                        "\n" +
                        "FPS:" +
                        EV_sync_update.fps_last
                    );
                  },
                  0,
                  0
                );
              }
              if (x.facemesh) {
                Ee.worker_onmessage({ data: x.facemesh });
              }
              if (K && Ee.worker_initialized) {
                let e = {
                  posenet: K,
                  w: ke.video_canvas.width,
                  h: ke.video_canvas.height,
                  flip_canvas: ke.display_flipped,
                };
                if (x.handpose && x.handpose.length) e.handpose = x.handpose;
                if (x.facemesh) e.facemesh = x.facemesh.faces;
                if (Pe.use_holistic || !Ee.enabled) {
                  e.draw_canvas = true;
                  if (self.FacemeshAT) {
                    e.canvas = ke.video_canvas_facemesh;
                  } else if (
                    !ke.video_canvas_facemesh._offscreen &&
                    self.OffscreenCanvas
                  ) {
                    e.canvas =
                      ke.video_canvas_facemesh.transferControlToOffscreen();
                    ke.video_canvas_facemesh._offscreen = true;
                    console.log("(Facemesh: use offscreen canvas)");
                  }
                }
                xe.postMessage(e, e.canvas ? [e.canvas] : undefined);
              }
              Oe = 0;
            }
            tt();
          };
