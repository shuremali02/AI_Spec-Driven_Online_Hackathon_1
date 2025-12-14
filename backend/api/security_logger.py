import logging
from datetime import datetime
from typing import Dict, Any, Optional
import json
from enum import Enum

class SecurityEventType(Enum):
    API_ACCESS = "api_access"
    AUTHENTICATION = "authentication"
    AUTHORIZATION = "authorization"
    INPUT_VALIDATION = "input_validation"
    RATE_LIMIT = "rate_limit"
    ERROR = "error"
    DATA_ACCESS = "data_access"

class SecurityLogger:
    """
    Security logger for audit purposes
    """
    def __init__(self, logger_name: str = "security"):
        self.logger = logging.getLogger(logger_name)
        self.logger.setLevel(logging.INFO)

        # Only add handler if it doesn't exist to avoid duplicates
        if not self.logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            handler.setFormatter(formatter)
            self.logger.addHandler(handler)

    def log_event(
        self,
        event_type: SecurityEventType,
        user_id: Optional[str] = None,
        ip_address: Optional[str] = None,
        endpoint: Optional[str] = None,
        status: Optional[str] = None,
        details: Optional[Dict[str, Any]] = None
    ):
        """
        Log a security event
        """
        event_data = {
            "timestamp": datetime.utcnow().isoformat(),
            "event_type": event_type.value,
            "user_id": user_id,
            "ip_address": ip_address,
            "endpoint": endpoint,
            "status": status,
            "details": details or {}
        }

        self.logger.info(json.dumps(event_data))

    def log_api_access(
        self,
        user_id: str,
        ip_address: str,
        endpoint: str,
        method: str,
        status_code: int,
        details: Optional[Dict[str, Any]] = None
    ):
        """
        Log API access event
        """
        self.log_event(
            event_type=SecurityEventType.API_ACCESS,
            user_id=user_id,
            ip_address=ip_address,
            endpoint=f"{method} {endpoint}",
            status=f"success" if 200 <= status_code < 300 else f"error_{status_code}",
            details=details
        )

    def log_authentication(
        self,
        user_id: str,
        ip_address: str,
        success: bool,
        details: Optional[Dict[str, Any]] = None
    ):
        """
        Log authentication event
        """
        self.log_event(
            event_type=SecurityEventType.AUTHENTICATION,
            user_id=user_id,
            ip_address=ip_address,
            status="success" if success else "failure",
            details=details
        )

    def log_authorization(
        self,
        user_id: str,
        ip_address: str,
        endpoint: str,
        success: bool,
        details: Optional[Dict[str, Any]] = None
    ):
        """
        Log authorization event
        """
        self.log_event(
            event_type=SecurityEventType.AUTHORIZATION,
            user_id=user_id,
            ip_address=ip_address,
            endpoint=endpoint,
            status="granted" if success else "denied",
            details=details
        )

    def log_input_validation(
        self,
        user_id: str,
        ip_address: str,
        endpoint: str,
        validation_passed: bool,
        details: Optional[Dict[str, Any]] = None
    ):
        """
        Log input validation event
        """
        self.log_event(
            event_type=SecurityEventType.INPUT_VALIDATION,
            user_id=user_id,
            ip_address=ip_address,
            endpoint=endpoint,
            status="passed" if validation_passed else "failed",
            details=details
        )

    def log_rate_limit(
        self,
        user_id: str,
        ip_address: str,
        endpoint: str,
        details: Optional[Dict[str, Any]] = None
    ):
        """
        Log rate limit event
        """
        self.log_event(
            event_type=SecurityEventType.RATE_LIMIT,
            user_id=user_id,
            ip_address=ip_address,
            endpoint=endpoint,
            status="exceeded",
            details=details
        )

    def log_error(
        self,
        user_id: Optional[str],
        ip_address: Optional[str],
        endpoint: Optional[str],
        error_type: str,
        details: Optional[Dict[str, Any]] = None
    ):
        """
        Log error event
        """
        self.log_event(
            event_type=SecurityEventType.ERROR,
            user_id=user_id,
            ip_address=ip_address,
            endpoint=endpoint,
            status=error_type,
            details=details
        )

    def log_data_access(
        self,
        user_id: str,
        ip_address: str,
        data_type: str,
        action: str,
        success: bool,
        details: Optional[Dict[str, Any]] = None
    ):
        """
        Log data access event
        """
        self.log_event(
            event_type=SecurityEventType.DATA_ACCESS,
            user_id=user_id,
            ip_address=ip_address,
            endpoint=f"{action} {data_type}",
            status="success" if success else "failure",
            details=details
        )

# Global security logger instance
security_logger = SecurityLogger()