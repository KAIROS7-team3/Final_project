# ADR — 데이터 / DB

> 참조: [인덱스](index.md)
> DB 논리 스키마 상세 → [`docs/db-schema.md`](../db-schema.md)

---

## ADR-008: DB 엔진 — SQLite WAL

- **결정**: SQLite WAL 모드
- **이유**: 단일 머신·단일 프로세스 환경, 공구 9종 수준의 경량 워크로드에 충분. 별도 서버 불필요, 파일 하나로 배포·백업 단순. WAL 모드로 읽기(대시보드)·쓰기(이벤트 로그) 동시 처리
- **제약**: HP ProBook 대시보드는 NFS 위 SQLite 직접 접근 금지 → HTTP API 또는 별도 read 경로 경유
- **물리 DDL 위치**: `db_core/schema.sql`, 마이그레이션 → `db_core/migrations/`

---

## 미결 사항

| # | 항목 | 결정 시점 |
|---|------|----------|
| 34 | 로그 파일 보존 기간 및 외부 집계 도구 (ELK / Loki / 없음) | Phase 7 전 |
