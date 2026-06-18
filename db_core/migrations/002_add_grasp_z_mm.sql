-- Migration 002: tools 테이블에 grasp_z_mm 컬럼 추가
-- 공구별 파지 Z 좌표 (mm, DSR BASE 좌표계) — 사물함내부_객체별base좌표.tw 기준
ALTER TABLE tools ADD COLUMN grasp_z_mm REAL;

-- 6종 공구 초기값 seed (tool_id는 기존 DB seed와 동일한 값 사용)
UPDATE tools SET grasp_z_mm = 97.32  WHERE tool_id LIKE '%ratchet_wrench%';
UPDATE tools SET grasp_z_mm = 99.87  WHERE tool_id LIKE '%utility_knife%';
UPDATE tools SET grasp_z_mm = 116.69 WHERE tool_id LIKE '%socket_19mm%';
UPDATE tools SET grasp_z_mm = 49.59  WHERE tool_id LIKE '%multi_tool%';
UPDATE tools SET grasp_z_mm = 42.65  WHERE tool_id LIKE '%screwdriver%';
UPDATE tools SET grasp_z_mm = 53.4   WHERE tool_id LIKE '%spanner_16mm%';
